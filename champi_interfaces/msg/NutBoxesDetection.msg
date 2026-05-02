# Detection result for a cluster of 4 nutboxes
# Pose is relative to base_link frame

std_msgs/Header header

# Pose of the center of the 4-box cluster in base_link frame
# position.z == -1.0 means no valid detection
geometry_msgs/Pose pose

# Colors of the 4 individual nutboxes (left to right when facing them)
# Use the constants below
uint8 COLOR_UNKNOWN = 0
uint8 COLOR_BLUE = 1
uint8 COLOR_YELLOW = 2

uint8[4] colors

