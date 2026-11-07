# Offline dev image: ROS 2 Humble (same distro as the drone), arm64 like the Jetson,
# with the planner's build deps (Eigen 3.4 + OMPL 1.7, as in the skybeta image).
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake libeigen3-dev ros-humble-ompl \
      python3-numpy python3-scipy python3-pytest \
      ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
