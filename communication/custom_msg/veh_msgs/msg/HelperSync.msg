int32               time_step           # timestamp of the message
int32               vehicle_id          # vehicle ID
bool                is_fallback         # true if vehicle cannot find a feasible trajectory and thus will take fallback
int32[]             vehs_fallback       # vehicles that need to take fallbacks
int32[]             predicted_trims     # predicted trims in the prediction horizon
int32[]             predicted_lanelets  # predicted lanelets in the prediction horizon
veh_msgs/PoseXy[]   predicted_areas     # predicted occupied areas in the prediction horizon
veh_msgs/PoseXy[]   reachable_sets      # reachable sets in the prediction horizon
float64[]           states              # 4-by-1 vector, vehicle states [x,y,yaw,speed]
bool                is_reading_finished # vehicles are only allowed to send messages to their topic if the value of `is_reading_finished` of the latest messages of all vehicles are true.
                                        # In this way we can avoid the needed messages are not available before reading since vehicles can only access the latest message of other vehicles. 