import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_experiment'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tb3 disturbance sim',
    maintainer_email='noreply@example.com',
    description='Ground-truth bridge, clearance evaluator, goal client and A/B '
                'experiment launch for TurtleBot3 state-estimation study.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gt_bridge_node = tb3_experiment.gt_bridge_node:main',
            'evaluator_node = tb3_experiment.evaluator_node:main',
            'goal_pose_client = tb3_experiment.goal_pose_client:main',
        ],
    },
)
