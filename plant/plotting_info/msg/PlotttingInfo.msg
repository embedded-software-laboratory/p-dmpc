float64[]                       trajectory_predictions      # L x 4
float64[]                       ref_trajectory              # 1 x Hp x 2
int32                           priorities                  # 1
int32                           n_obstacles                 # 1 (= 0)
int32                           n_dynamic_obstacles         # 1 (= 0)
# dyamic_obstacles                                          (left out)
# dyamic_obstacles_shape                                    (left out)
# reachable_sets                                            (left out/polyshape)
int32                           step                        # 1
int32                           vehicle_indices             # 1
int32                           tick_now                    # 1 (= 1)
# lanelet_crossing_areas                                    (left out)
float64[]                       weighted_coupling_reduced   # N x N
int32[]                         directed_coupling           # N x N
int32                           belonging_vector            # 1
CouplingInfo[]                  coupling_info               # N x N
