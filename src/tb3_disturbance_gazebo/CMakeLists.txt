cmake_minimum_required(VERSION 3.8)
project(tb3_disturbance_gazebo)

find_package(ament_cmake REQUIRED)

# Install simulation assets and launch files.
install(DIRECTORY
  models
  worlds
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Also expose the model/media directory to Gazebo via the ament environment hook
# defined by the <gazebo_ros> export tags in package.xml.

ament_package()
