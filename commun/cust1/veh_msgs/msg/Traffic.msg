int32               time_step                   # timestamp of the message
int32               vehicle_id                  # vehicle ID
bool                is_fallback                 # true if vehicle cannot find a feasible trajectory and thus will take fallback
veh_msgs/Pos                   current_pos
int32               current_trim_index          # current trim
int32[]             predicted_lanelets          # predicted lanelets in the prediction horizon
veh_msgs/PoseXy[]   predicted_areas             # predicted occupied areas of current time step
veh_msgs/PoseXy[]   reachable_sets              # reachable sets in the prediction horizon
