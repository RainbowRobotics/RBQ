# World-axis-aligned 2.5D grid (ObstacleAvoidance). Cell (r, c) is height[r * cols + c].

std_msgs/Header header
int32 rows
int32 cols
float32 cell_size               # [m]
float32 origin_x                # world x of cell (0, 0) [m]
float32 origin_y
float32[] height                # [m]
uint8[] valid                   # 0/1 — height of an invalid cell is meaningless
