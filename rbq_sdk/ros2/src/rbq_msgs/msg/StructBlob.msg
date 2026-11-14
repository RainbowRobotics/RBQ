# A C++ struct carried as raw bytes between RBQ processes: internal state and commands that have no message of
# their own yet. type_name names the struct; data is its memory image (x86-64 / aarch64 little endian).

std_msgs/Header header
string type_name
uint8[] data
