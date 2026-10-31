# A point cloud as the vision apps exchange it. Fields x, y, z, intensity (float32). The cloud alone is also
# mirrored as sensor_msgs/PointCloud2 (points) while anyone subscribes to it.

sensor_msgs/PointCloud2 cloud
uint8 kind                      # heightmap cloud kind (map 0, edge 1, stair 2, stair edge 3); 0 for sensor clouds
float64[16] body_from_sensor    # 4x4 sensor -> body transform, column-major
