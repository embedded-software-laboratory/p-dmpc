int32               time_step                   # timestamp of the message
int32               vehicle_id                  # vehicle ID
int32[]             vehs_fallback               # index of vehicles that need to take fallbacks
int32[]             predicted_trims             # predicted trims in the prediction horizon
int32[]             predicted_lanelets          # predicted lanelets in the prediction horizon
veh_msgs/PoseXy[]   predicted_areas             # predicted occupied areas in the prediction horizon
