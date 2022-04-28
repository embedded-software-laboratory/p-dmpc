float64[]  state # array of data [4x1] = [x0; y0; yaw0; steering_angle0]
veh_msgs/Area[]     areas # predicted occupied areas
int32      trim_current # current trim
int32      veh_id # vehicle ID
int32      lanelet_current # current lanelet in which the vehicle drives
int32[]    lanelets_future # future lanelets that the vehicle plan to drive in