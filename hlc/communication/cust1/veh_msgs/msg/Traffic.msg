int32               time_step                   # timestamp of the message
int32               vehicle_id                  # vehicle ID
bool                is_fallback                 # true if vehicle cannot find a feasible trajectory and thus will take fallback
veh_msgs/PoseXy     current_pose                # current Pose
int32               current_trim_index          # current trim
int32[]             predicted_lanelets          # predicted lanelets in the prediction horizon
veh_msgs/PoseXy[]   occupied_areas              # occupied areas of current time step
veh_msgs/PoseXy[]   reachable_sets              # reachable sets in the prediction horizon
