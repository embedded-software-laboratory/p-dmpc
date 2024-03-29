function [iter] = coupling_adjacency(scenario, iter)
    % COUPLING_ADJACENCY returns the adjacency matrix based on predicted
    % trajectories. If the future trajectories are in two ajacent lanelets and
    % vehicles are within defined distance, the vehicles are considered to be adjacent.

    nVeh = size(iter.reference_trajectory_points, 1);
    Hp = size(iter.reference_trajectory_points, 2);
    adjacency = zeros(nVeh, nVeh);
    adjacent = scenario.adjacency_lanelets;

    for i = 1:(nVeh - 1)
        ref_points_i = reshape(iter.reference_trajectory_points(i, :, :), Hp, 2);
        predicted_lanelets_i = iter.predicted_lanelets{i};

        for j = (i + 1):nVeh
            stop_flag = false;
            ref_points_j = reshape(iter.reference_trajectory_points(j, :, :), Hp, 2);
            predicted_lanelets_j = iter.predicted_lanelets{j};

            for k_i = 1:length(predicted_lanelets_i)

                for k_j = 1:length(predicted_lanelets_j)
                    stop_flag = false;
                    % check if lanelets are adjacent
                    if adjacent(predicted_lanelets_i(k_i), predicted_lanelets_j(k_j)) == 1
                        % check if predicted trajectories are within defined distance
                        for k = 1:Hp
                            dist = norm((ref_points_i(k, :) - ref_points_j(k, :)), 2);

                            if dist < 1.4
                                adjacency(i, j) = 1;
                                adjacency(j, i) = 1;
                                stop_flag = true;
                            end

                            if stop_flag
                                break
                            end

                        end

                    end

                    if stop_flag
                        break
                    end

                end

                if stop_flag
                    break
                end

            end

        end

    end

    iter.adjacency = adjacency;
end
