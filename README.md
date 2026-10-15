# Monocular Visual Odometry

This project estimates camera motion from a monocular image sequence and compares the estimated trajectory with ground-truth poses. The helper module detects FAST features, tracks them with pyramidal Lucas–Kanade optical flow, and estimates relative motion from an essential matrix.

## Contents

- `mono_vo.py` contains feature detection/tracking and the `monoVO(frame_0, frame_1, MIN_NUM_FEAT)` helper.
- `src.py` is the driver scaffold. It describes the intended frame subscription, ground-truth extraction, trajectory plotting, and Absolute Trajectory Error (ATE) calculation.
- `dataset/pose/00.txt` contains pose data.
- `dataset/sequence/image_0/` is the image sequence, excluded from Git to avoid committing hundreds of megabytes.
- The original ROS bag and ZIP archive are large local data copies and should not be committed; the bag exceeds GitHub's 100 MB per-file limit.

## Requirements

Python 3 is recommended for a fresh setup, with NumPy, OpenCV, and Matplotlib. `src.py` also imports `rospy`, though the visible scaffold does not yet implement its ROS pipeline. The image sequence and pose data must be available under the expected relative paths.

## Current state and running

The VO helper is present, but `src.py` is incomplete: `plotTraj` is a stub, `computeError` currently returns zero, and the end-to-end image/pose processing described in the comments is not implemented. Therefore there is no complete run command yet. Complete the driver and provide the data before attempting evaluation.

`mono_vo.py` expects grayscale image arrays and returns a rotation matrix, translation direction, and a validity flag. Monocular translation has unknown metric scale without additional information; account for this when comparing trajectories.

## Data and attribution

Large image, bag, and ZIP assets are excluded from this project repository. Retrieve them from the original course source when needed. Confirm the dataset and handout redistribution terms before publishing copies.
