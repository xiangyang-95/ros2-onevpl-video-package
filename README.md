# Getting Started
This repo contains ROS2 package that utilize Intel OneVPL to encode and decode video stream

## Build
Run the following command to build the package
```bash
source /opt/ros/humble/setup.bash
colcon build
```

## Usage
Run the following command to run the package

### Video test stream publisher node
```bash
. install/setup.bash
ros2 run ros2-onevpl-video-package video-test-publisher <video-file-dir>

example: ros2 run ros2-onevpl-video-package video-test-publisher ./src/ros2-onevpl-video-package/data/hd_test_video.mp4
```

### Video stream encode node
```bash
TBA
```

### Video stream decode node
```bash
TBA
```