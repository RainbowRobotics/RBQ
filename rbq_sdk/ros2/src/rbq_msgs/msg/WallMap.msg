# Robot-centred, heading-up inflated wall grid (ObstacleAvoidance). Cells are 0 or 255.

std_msgs/Header header
int32 rows
int32 cols
float32 cell_size               # [m]
float64[16] body_from_map       # 4x4, column-major
uint8[] cells
