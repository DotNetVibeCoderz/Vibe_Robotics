# This represents a 2-D grid map
std_msgs/Header header

# MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).
# Cell (1, 0) will be listed second, representing the next cell in the x direction.
# Occupancy probabilities are in the range [0,100]. Unknown is -1.
int8[] data
