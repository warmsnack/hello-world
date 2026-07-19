cmake_minimum_required(VERSION 3.8)
project(tb3_analysis)

find_package(ament_cmake REQUIRED)

# Install analysis scripts as executables.
install(PROGRAMS
  scripts/plot_cmd_vel.py
  scripts/run_evo.sh
  scripts/run_ab_experiment.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
