# One LiDAR sweep with its IMU samples. Fields x, y, z, reflectivity, alpha (per-point time ratio 0..1 for
# deskewing), all float32; header.stamp is the sweep start. Mirrored as sensor_msgs/PointCloud2 (points).

sensor_msgs/PointCloud2 cloud
float64 time_end                # sweep end, system time [s]
LidarImu[] imu
float64[16] body_from_sensor    # 4x4 points -> body, column-major
float64[16] body_from_imu       # 4x4 IMU -> body, column-major
uint8 time_type                 # 0 internal clock, 1 PTP synchronized
