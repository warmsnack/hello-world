#!/usr/bin/env python3
"""Phase 5 - Control-response analysis.

Reads /cmd_vel (geometry_msgs/Twist) from one or two rosbag2 (SQLite3) files and
plots the linear/angular velocity, acceleration and JERK. Comparing Mode A
(ground-truth localization) against Mode B (EKF) shows how much extra motor
"jerk" (sudden accel/decel) the noisy estimator induces in the controller.

Examples
--------
  # Compare A vs B and save a figure:
  python3 plot_cmd_vel.py --bag-b ~/tb3_eval/rosbag_mode_B \
                          --bag-a ~/tb3_eval/rosbag_mode_A \
                          --out ~/tb3_eval/cmd_vel_compare.png

  # Single bag + CSV export:
  python3 plot_cmd_vel.py --bag-b ~/tb3_eval/rosbag_mode_B \
                          --csv ~/tb3_eval/cmd_vel_B.csv
"""

import argparse
import csv
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover
    SequentialReader = None


def read_cmd_vel(bag_uri, topic='/cmd_vel'):
    """Return (t[s], vx, wz) numpy arrays for `topic` in a rosbag2 dir."""
    if SequentialReader is None:
        raise RuntimeError(
            'rosbag2_py / rclpy not importable. Source your ROS 2 workspace '
            'before running this script.')

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_uri, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise RuntimeError(
            f"Topic '{topic}' not in bag. Available: {list(type_map)}")
    msg_type = get_message(type_map[topic])

    t0 = None
    ts, vxs, wzs = [], [], []
    while reader.has_next():
        name, data, stamp_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if t0 is None:
            t0 = stamp_ns
        ts.append((stamp_ns - t0) * 1e-9)
        vxs.append(msg.linear.x)
        wzs.append(msg.angular.z)

    return np.array(ts), np.array(vxs), np.array(wzs)


def derivatives(t, v):
    """Return acceleration (dv/dt) and jerk (d2v/dt2) via np.gradient."""
    if len(t) < 3:
        return np.zeros_like(v), np.zeros_like(v)
    accel = np.gradient(v, t)
    jerk = np.gradient(accel, t)
    return accel, jerk


def export_csv(path, t, vx, wz):
    accel_x, jerk_x = derivatives(t, vx)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'vx', 'wz', 'accel_x', 'jerk_x'])
        for i in range(len(t)):
            w.writerow([f'{t[i]:.4f}', f'{vx[i]:.5f}', f'{wz[i]:.5f}',
                        f'{accel_x[i]:.5f}', f'{jerk_x[i]:.5f}'])
    print(f'Wrote {path}')


def summarize(label, t, vx, wz):
    accel_x, jerk_x = derivatives(t, vx)
    print(f'--- Mode {label} ---')
    print(f'  samples          : {len(t)}')
    print(f'  duration         : {t[-1] - t[0]:.2f} s' if len(t) else '  no data')
    print(f'  |vx| max         : {np.max(np.abs(vx)):.3f} m/s' if len(vx) else '')
    print(f'  |accel_x| max    : {np.max(np.abs(accel_x)):.3f} m/s^2'
          if len(accel_x) else '')
    print(f'  |jerk_x| RMS     : {np.sqrt(np.mean(jerk_x**2)):.3f} m/s^3'
          if len(jerk_x) else '')
    print(f'  |jerk_x| max     : {np.max(np.abs(jerk_x)):.3f} m/s^3'
          if len(jerk_x) else '')


def plot(datasets, out_path):
    if plt is None:
        print('matplotlib not available; skipping plot.', file=sys.stderr)
        return
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    for label, (t, vx, wz) in datasets.items():
        if len(t) == 0:
            continue
        accel_x, jerk_x = derivatives(t, vx)
        axes[0].plot(t, vx, label=f'Mode {label}')
        axes[1].plot(t, accel_x, label=f'Mode {label}')
        axes[2].plot(t, jerk_x, label=f'Mode {label}')

    axes[0].set_ylabel('linear vx [m/s]')
    axes[1].set_ylabel('accel_x [m/s^2]')
    axes[2].set_ylabel('jerk_x [m/s^3]')
    axes[2].set_xlabel('time [s]')
    axes[0].set_title('cmd_vel forward velocity / acceleration / jerk (A vs B)')
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print(f'Saved figure to {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag-a', help='rosbag2 dir for Mode A (ground truth).')
    ap.add_argument('--bag-b', help='rosbag2 dir for Mode B (EKF).')
    ap.add_argument('--topic', default='/cmd_vel')
    ap.add_argument('--out', default='cmd_vel_compare.png',
                    help='Output PNG for the comparison plot.')
    ap.add_argument('--csv', default=None,
                    help='Optional CSV export path (uses whichever bag is set, '
                         'preferring Mode B).')
    args = ap.parse_args()

    if not args.bag_a and not args.bag_b:
        ap.error('Provide at least one of --bag-a / --bag-b.')

    datasets = {}
    if args.bag_a:
        datasets['A'] = read_cmd_vel(args.bag_a, args.topic)
    if args.bag_b:
        datasets['B'] = read_cmd_vel(args.bag_b, args.topic)

    for label, (t, vx, wz) in datasets.items():
        summarize(label, t, vx, wz)

    if args.csv:
        label = 'B' if 'B' in datasets else 'A'
        export_csv(args.csv, *datasets[label])

    plot(datasets, args.out)


if __name__ == '__main__':
    main()
