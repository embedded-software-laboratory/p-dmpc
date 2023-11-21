float64[]   trajectory_predictions          # L x 4
float64[]   ref_trajectory                  # 1 x Hp x 2
int32       priorities                      # 1
int32       n_obstacles                     # 1 (= 0)
int32       n_dynamic_obstacles             # 1 (= 0)
int32       step                            # 1
int32       veh_indices                     # 1
int32       tick_now                        # 1 (= 1)
float64[]   weighted_coupling_reduced       # N x N
uint8[]     directed_coupling               # N x N
uint8[]     directed_coupling_sequential    # N x N
int32[]     is_virtual_obstacle             # N x N (boolean matrix extracted from coupling_info)
