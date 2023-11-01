int32               time_step                   # timestamp of the message
int32               vehicle_id                  # vehicle ID
int32[]             vehs_fallback               # index of vehicles that need to take fallbacks
int32               priority_permutation        # permutation index for multiple priorities
veh_msgs/PoseXy[]   predicted_areas             # predicted occupied areas in the prediction horizon
