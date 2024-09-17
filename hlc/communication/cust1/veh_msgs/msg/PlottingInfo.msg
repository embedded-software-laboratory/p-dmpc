float64[]   trajectory_predictions          # 3 x Hp
float64[]   ref_trajectory                  # 1 x Hp x 2
float64[3]  state                           # 3 x 1
int32       n_obstacles
int32       step
float64     time_seconds
int32       vehicle_indices
uint8[]     directed_coupling               # N x N
uint8[]     directed_coupling_sequential    # N x N
float64[]   weighted_coupling               # N x N
