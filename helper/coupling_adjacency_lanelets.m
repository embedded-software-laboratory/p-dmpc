function adjacency = coupling_adjacency_lanelets(lanelet_idx,collision)
% COUPLING_ADJACENCY_LANELETS   Computes undirected coupling adjacency matrix based on the lanelet indices.

    nVeh = size(lanelet_idx,1);

    adjacency = zeros(nVeh,nVeh);

    % take all pairwise different combinations
    for i = 1 : nVeh-1
        % get lanelet indices for vehicle i
        lanelets_i = lanelet_idx(i,:);
        for j = nVeh : -1 : i+1
            stop_flag = false;
            % get lanelet indices for vehicle j
            lanelets_j = lanelet_idx(j,:);
            for li = lanelets_i
                for lj = lanelets_j
                    % is there a collision pair among these lanelet index seqeunce
                    if collision(li,lj)
                        adjacency(i,j) = 1;
                        adjacency(j,i) = 1;
                        % no need to check any other pair for these two vehicles
                        stop_flag = true;
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

end

