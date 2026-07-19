# TurtleBot3 Waffle · 상태 추정에 따른 동적 장애물 회피 실험 (ROS 2 + Nav2 + Gazebo)

가혹한 물리 환경(센서 노이즈 · 바퀴 슬립)과 동적 장애물이 있는 Gazebo 시뮬레이션에서,
**상태 추정 알고리즘(A: Ground Truth vs. B: EKF)** 에 따라 Nav2 의 동적 장애물 회피 성능이
어떻게 달라지는지를 정량적으로 비교(A/B 테스트)하기 위한 워크스페이스입니다.

> **대상 스택**: ROS 2 **Humble** + **Gazebo Classic 11** + TurtleBot3 **Waffle** + Nav2 + `robot_localization`.
> 이 조합은 과제에서 쓰는 모든 API(`libgazebo_ros_diff_drive.so`, `<publish_odom_tf>`,
> `/gazebo/model_states`, `<actor>` 등)가 그대로 동작하는 정식 조합입니다.

---

## 핵심 아이디어

Gazebo 는 기본적으로 "완벽한" 오도메트리(정답 TF)를 발행합니다. 이를 끊고,
- **바퀴 마찰을 극단적으로 낮춰(mu=0.05)** cmd_vel 과 실제 이동 사이에 괴리(슬립)를 만들고,
- **IMU/오도메트리에 가우시안 노이즈**를 주입하여 추정 오차를 유발한 뒤,
- 그 위에서 **두 가지 상태 추정 방식**을 스위치 하나(`use_gt`)로 바꿔 실험합니다.

| 모드 | `use_gt` | `odom → base_footprint` TF 발행자 | 의미 |
|------|----------|------------------------------------|------|
| **A** | `true`  | `gt_bridge_node` (Gazebo 절대 좌표) | "완벽한 추정기" 기준선 |
| **B** | `false` | `robot_localization` EKF            | 노이즈/슬립을 겪는 실제 추정 |

두 모드에서 나머지 파이프라인(월드, Nav2, 목표점, 평가기, rosbag)은 **완전히 동일**하므로,
회피 성능 차이는 오롯이 상태 추정기 차이로 귀결됩니다.

### TF 트리

```
map ──(static identity, launch)── odom ──(A: gt_bridge / B: EKF)── base_footprint
                                                                     ├── base_link ── (wheels, imu_link, base_scan …)
```

`diff_drive` 플러그인은 `<publish_odom_tf>false</publish_odom_tf>` 로 설정되어
`odom → base_footprint` 를 **절대 발행하지 않습니다**. 그 자리를 A/B 추정기가 채웁니다.

### 주요 토픽

| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller | 제어 명령 |
| `/odom_unfiltered` | `nav_msgs/Odometry` | diff_drive (ENCODER) | **슬립을 겪는 원본** 휠 오도메트리 |
| `/imu` | `sensor_msgs/Imu` | imu 센서 플러그인 | **가우시안 노이즈** 포함 |
| `/odom` | `nav_msgs/Odometry` | EKF(B) | 융합된 추정치 (Nav2 입력) |
| `/gt_odom` | `nav_msgs/Odometry` | `gt_bridge_node` | **정답 궤적** (평가/evo 기준) |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | `gazebo_ros_state` | 모든 모델 절대 좌표 |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR | 코스트맵 장애물 감지 |

---

## 패키지 구성 (5개 Phase 매핑)

```
src/
├── tb3_disturbance_gazebo/        # Phase 1: 물리 환경 및 외란 모델링
│   ├── models/turtlebot3_waffle/model.sdf   # IMU 노이즈 + mu=0.05 슬립 + publish_odom_tf=false
│   ├── worlds/disturbance.world             # 보행자 actor + 이동 실린더 actor + model_states
│   ├── launch/disturbance_world.launch.py   # Gazebo 기동 + 로봇 스폰
│   └── rviz/experiment.rviz
│
├── tb3_state_estimation/          # Phase 2: 상태 추정 파이프라인
│   ├── config/ekf.yaml                      # 휠 vx + IMU vyaw 융합
│   └── launch/ekf.launch.py
│
├── tb3_experiment/                # Phase 3 & 4: GT 브릿지 · 평가기 · 통합 실행
│   ├── tb3_experiment/gt_bridge_node.py     # /gazebo/model_states → /gt_odom (+TF)
│   ├── tb3_experiment/evaluator_node.py     # 최소 Clearance → CSV
│   ├── tb3_experiment/goal_pose_client.py   # 5m 직진 목표 자동 하달 (Nav2 Action)
│   ├── config/nav2_params.yaml              # map-less Nav2 (global_frame=odom)
│   └── launch/experiment.launch.py          # use_gt:=true/false + rosbag record -a
│
└── tb3_analysis/                  # Phase 5: 정량 분석
    ├── scripts/run_evo.sh                   # evo_ape(ATE) + evo_rpe(--delta, spike)
    ├── scripts/plot_cmd_vel.py              # cmd_vel jerk(급가감속) A/B 비교
    └── scripts/run_ab_experiment.sh         # A/B 자동 실행 + 분석 일괄
```

---

## Phase 별 상세

### Phase 1 — 물리 환경 및 외란 모델링
- **센서 노이즈**: `model.sdf` 의 IMU `<angular_velocity>`/`<linear_acceleration>` 에
  가우시안 `<stddev>`·`<bias_mean>` 추가. LiDAR 에도 `gaussian` 노이즈 부여.
- **마찰(Slip) 조작**: 양 바퀴 `<collision>` 의 `<mu1>`, `<mu2>` 를 **0.05** 로 낮추고
  `<slip1>/<slip2>` 를 높여 cmd_vel ↔ 실제 기구학 괴리 유발.
- **동적 장애물**: `<actor>` 로 (1) 로봇 경로(x≈2.5 m)를 횡단하는 **보행자**,
  (2) 폴리라인을 도는 **이동 실린더** 배치.

### Phase 2 — 상태 추정 파이프라인 재구축
- **완벽한 TF 차단**: `diff_drive` 의 `<publish_odom_tf>false</publish_odom_tf>`,
  오도메트리 소스를 `ENCODER(1)` 로 하여 슬립이 오도메트리 드리프트로 나타나게 함.
- **EKF 구성**: `ekf.yaml` 에서 `odom0`(=`/odom_unfiltered`) 은 **전진 속도 vx** 만,
  `imu0`(=`/imu`) 는 **요 각속도 vyaw** 만 융합(`two_d_mode: true`).
- **TF 재연결**: EKF 가 `odom → base_footprint` 를 발행(`/odom` 로 리매핑된 필터 출력).

### Phase 3 — Ground Truth 파이프라인 및 평가기
- `gt_bridge_node.py`: `/gazebo/model_states` 구독 → 로봇 절대 좌표를
  `nav_msgs/Odometry` 로 변환해 **`/gt_odom`** 발행. Mode A 에서는 TF 도 발행.
- `evaluator_node.py`: 로봇–각 동적 장애물 간 유클리드 거리를 10 Hz 로 계산,
  **최소 Clearance** 를 CSV(`clearance_A.csv`/`clearance_B.csv`) 로 로깅.

### Phase 4 — 통합 주행 실험 및 데이터 로깅
- `experiment.launch.py`: **`use_gt:=true/false`** 인자로 EKF/GT-브릿지 TF 발행 전환.
- `goal_pose_client.py`: RViz 수동 지정 없이 **5 m 직진 목표**를 Nav2 Action 으로 자동 하달.
- `record:=true` 시 **`ros2 bag record -a` (SQLite3)** 로 `/tf`, `/cmd_vel`, `/odom`, `/gt_odom` 등 녹화.

### Phase 5 — 정량적 데이터 분석 및 시각화
- `run_evo.sh`: `evo_ape` 로 장기 이탈(**ATE**), `evo_rpe --delta 1 -f` 로
  노이즈성 **단기 스파이크(RPE)** 시각화 (`/gt_odom` 기준, `/odom` 추정).
- `plot_cmd_vel.py`: rosbag 의 `/cmd_vel` → 속도·가속도·**Jerk(급가감속)** 를
  A/B 비교 플롯 및 CSV 로 추출.

---

## 빌드 & 실행

### 사전 준비 (Ubuntu 22.04 / ROS 2 Humble 가정)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization
pip install evo        # Phase 5 궤적 분석용
```

### 빌드

```bash
# 이 저장소를 콜콘 워크스페이스로 사용
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
colcon build --symlink-install
source install/setup.bash
```

### Mode B (EKF) 실행 + 녹화

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch tb3_experiment experiment.launch.py use_gt:=false record:=true
```

### Mode A (Ground Truth) 실행 + 녹화

```bash
ros2 launch tb3_experiment experiment.launch.py use_gt:=true record:=true
```

주요 launch 인자: `use_gt`(A/B), `gui`(GUI on/off), `record`(rosbag),
`goal_x`(기본 5.0 m), `csv_dir`(결과 저장 경로, 기본 `~/tb3_eval`).

### A/B 자동 실행 + 분석 일괄

```bash
ros2 run tb3_analysis run_ab_experiment.sh 60 ~/tb3_eval
```

### 개별 분석

```bash
# 궤적 오차 (ATE / RPE)
ros2 run tb3_analysis run_evo.sh ~/tb3_eval/rosbag_mode_B /gt_odom /odom

# cmd_vel jerk A/B 비교
ros2 run tb3_analysis plot_cmd_vel.py \
    --bag-a ~/tb3_eval/rosbag_mode_A --bag-b ~/tb3_eval/rosbag_mode_B \
    --out ~/tb3_eval/cmd_vel_compare.png
```

### 산출물 (`~/tb3_eval/`)
- `clearance_A.csv`, `clearance_B.csv` — 시간별 거리 & 최소 Clearance
- `rosbag_mode_A/`, `rosbag_mode_B/` — SQLite3 rosbag
- `*_evo/ape_plot.pdf`, `rpe_plot.pdf` — 궤적 오차 시각화
- `cmd_vel_compare.png` — 모터 급가감속(Jerk) 비교

---

## 참고 / 한계

- 본 저장소는 **소스 워크스페이스**입니다. Gazebo Classic 은 GUI/물리 시뮬레이터라
  헤드리스 CI 환경에서 자동 검증이 어려워, 여기서는 파이썬/런치 문법, XML(SDF/World),
  YAML 유효성까지 정적 검증했습니다. 실제 주행 검증은 위 실행 절차대로 로컬(GPU/디스플레이가
  있는 ROS 2 Humble 환경)에서 진행하세요.
- `map-less` 구성(양 코스트맵 `global_frame: odom`, rolling window)으로 `map_server`/AMCL
  없이도 동적 장애물 회피가 동작합니다. `map → odom` 은 launch 의 static identity TF 로 연결합니다.
- Gazebo Classic 이 EOL 인 환경(예: Ubuntu 24.04/ROS 2 Jazzy)에서는 신형 `gz-sim` 으로의
  포팅(플러그인 이름/토픽 변경)이 필요합니다.
