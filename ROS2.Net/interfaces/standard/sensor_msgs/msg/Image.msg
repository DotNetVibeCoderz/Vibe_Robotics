# This message contains an uncompressed image
std_msgs/Header header

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns
string encoding              # Encoding of pixels -- channel meaning, ordering, size
uint8 is_bigendian           # is this data bigendian?
uint32 step                  # Full row length in bytes
uint8[] data                 # actual matrix data, size is (step * rows)
