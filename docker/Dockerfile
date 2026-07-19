# ROS 2 Humble + Gazebo Classic 11 image for the TurtleBot3 dynamic-obstacle
# avoidance experiment workspace.
#
# The host VM runs Ubuntu 24.04, but this project targets Ubuntu 22.04 / ROS 2
# Humble / Gazebo Classic 11, so we run everything inside this container.
#
# The base image (osrf/ros:humble-desktop-full) already bundles ROS 2 Humble,
# RViz2 and Gazebo Classic 11. Here we add the extra ROS packages the workspace
# declares (turtlebot3, nav2, robot_localization) plus the Phase 5 analysis
# tooling (evo). The colcon workspace itself is NOT built here; it is mounted at
# runtime and built with `colcon build` so source edits are picked up.
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle

# NOTE: do NOT use --no-install-recommends here. Nav2's lifecycle_manager loads
# libdiagnostic_updater.so at runtime; dropping recommended packages leaves that
# shared library missing and the whole Nav2 stack fails to activate. We also list
# ros-humble-diagnostic-updater explicitly as belt-and-suspenders.
RUN apt-get update && apt-get install -y \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-turtlebot3 \
      ros-humble-turtlebot3-msgs \
      ros-humble-turtlebot3-gazebo \
      ros-humble-turtlebot3-description \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-robot-localization \
      ros-humble-diagnostic-updater \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Phase 5 trajectory analysis tooling.
RUN pip3 install --no-cache-dir evo

WORKDIR /workspace
