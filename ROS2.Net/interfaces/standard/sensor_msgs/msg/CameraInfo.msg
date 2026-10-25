# This message defines meta information for a camera.
std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9]  k # 3x3 row-major matrix
float64[9]  r # 3x3 row-major matrix
float64[12] p # 3x4 row-major matrix
uint32 binning_x
uint32 binning_y
RegionOfInterest roi
