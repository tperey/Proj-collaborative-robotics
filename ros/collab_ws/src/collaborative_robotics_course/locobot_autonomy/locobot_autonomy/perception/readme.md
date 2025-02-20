`ros2 launch realsense2_camera rs_launch.py align_depth:=true`

- Subscribes to `/camera/color/image_raw` from the Intel RealSense D435.
- Converts the image to grayscale and detects apples using Hough Circles.
- Subscribes to `/odom` to get robot position and velocity.
- Logs detected apples with odometry data to understand the apple’s relative position.



