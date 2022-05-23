int32               time_step           # timestamp of the message
int32               vehicle_id          # vehicle ID
int32[]             predicted_trims     # predicted trims in the prediction horizon
int32[]             predicted_lanelets  # predicted lanelets in the prediction horizon
veh_msgs/PoseXy[]   predicted_areas     # predicted occupied areas in the prediction horizon
veh_msgs/PoseXy[]   reachable_sets      # reachable sets in the prediction horizon
