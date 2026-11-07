# Offline dev image for the localization packages: ROS 2 Humble + what the drone's stack image
# already carries for us (OpenCV, NumPy < 2). Build: docker build -t team-loc-dev:humble tools/loc_dev
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-opencv python3-pytest ros-humble-tf2-ros-py ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
