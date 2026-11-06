# One camera frame as the vision apps exchange it: the image plus what is needed to use it.
# Only the image is also mirrored as sensor_msgs/Image (image_raw) and sensor_msgs/CameraInfo (camera_info) for
# ROS tools, while anyone subscribes to those.

sensor_msgs/Image image
float32[4] intrinsics           # fx, fy, ppx, ppy [px]
float32[8] coeffs               # distortion coefficients
float32[12] extrinsics          # to the colour stream: rotation[9] (row-major), translation[3] [m]
float64[16] body_from_sensor    # 4x4 sensor -> body transform, column-major
