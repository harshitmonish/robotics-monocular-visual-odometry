# Monocular Visual Odometry

A ROS-based monocular visual odometry coursework project. It tracks FAST features between consecutive camera frames with pyramidal Lucas–Kanade optical flow, estimates relative camera motion from an essential matrix, and publishes estimated and KITTI ground-truth trajectories as RViz markers.

## Repository layout

```text
.
├── README.md
├── pattern.png                 # Calibration target image
└── src/                        # ROS catkin package (lab4)
    ├── mono_vo.py              # Feature tracking and relative pose estimation
    ├── src.py                  # VO ROS node and trajectory marker publishing
    ├── view_kitti.py           # KITTI image/pose publisher
    ├── src_cal.py              # Chessboard camera calibration utility
    ├── package.xml
    ├── CMakeLists.txt
    ├── dataset/pose/00.txt     # KITTI sequence 00 ground-truth poses
    ├── images/                 # Calibration photos
    └── rviz/                   # RViz configurations
```

The KITTI `sequence/image_0` directory is not present in this checkout. `view_kitti.py` expects the grayscale/color image sequence at `src/dataset/sequence/image_0/` and the pose file at `src/dataset/pose/00.txt`.

## Requirements

- ROS 1 with `rospy`, `cv_bridge`, `sensor_msgs`, `geometry_msgs`, and `visualization_msgs`
- A catkin workspace
- Python packages: NumPy, OpenCV, and Matplotlib (the calibration utility uses OpenCV)
- KITTI odometry sequence 00 images, placed at `src/dataset/sequence/image_0/`

The scripts also assume a KITTI camera calibration (`fx = 718.856`, `cx = 607.1928`, `cy = 185.2157`) in `mono_vo.py`.

## Build and run

Place this repository's `src` directory in a catkin workspace's `src` directory, then build and source the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

The publisher requires a ROS parameter `/fps`; the VO node requires `/src/fps`. With the image sequence installed and the workspace sourced, start the nodes in separate terminals:

```bash
rosparam set /fps 10
rosparam set /src/fps 10
rosrun src view_kitti.py
rosrun src src.py
```

The scripts currently live directly in the package directory and are not installed by `CMakeLists.txt`; if `rosrun` cannot find them, run them with Python from `src/` after sourcing ROS and the workspace:

```bash
python3 view_kitti.py
python3 src.py
```

`view_kitti.py` publishes `/kitti_image` (`sensor_msgs/Image`) and `/kitti_pose` (`geometry_msgs/TwistStamped`). The VO node subscribes to those topics and publishes `/kitti_Trajectory_marker` and `/Kitti_ground_Truth_marker`. The visualization helper `view_kitti.py` also publishes `/kitti_pose_marker`. RViz configuration files are in `src/rviz/`.

## Current implementation notes

- `mono_vo.py` returns relative rotation, translation direction, and a validity flag based on the minimum tracked-feature threshold. Monocular translation has unknown metric scale, so the estimate cannot directly match metric KITTI positions without scale handling.
- `view_kitti.py` loops over frames 0 through 1000 and therefore expects at least 1001 images and pose rows.
- `src.py` contains ROS callbacks and marker publishing, but `plotTraj` is unimplemented and `computeError` is a placeholder. Its `evaluate_error` routine is present but does not perform a standard aligned ATE calculation; treat its printed value as preliminary.
- `src_cal.py` detects a 7-by-6 chessboard in the JPGs under `src/images/` and prints the resulting camera matrix.

Large dataset archives and ROS bags are not included. Obtain the KITTI images and any course-provided assets from their original source, and check their redistribution terms before publishing copies.
