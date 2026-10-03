# Robot-centric 2.5D terrain elevation map (hyperdog_perception).
std_msgs/Header header        # frame_id: frame of the map (odom)
float32 resolution            # [m] cell size
uint32 width                  # number of cells along x
uint32 height                 # number of cells along y
float64 origin_x              # [m] corner of cell (0, 0) in header.frame_id
float64 origin_y
float32[] data                # terrain height [m], index = iy * width + ix; NaN: unknown
