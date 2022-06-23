function [belonging_vector, subgraphs_info] = graph_merging_algorithm(M, belonging_vector, subgraphs_info, max_num_CLs)
% GRAPH_MERGING_ALGORITHM Find connected subgraph pairs that can be merged to
% one bigger graph while still not exceeding the maximum allowed
% computation levels. This algorithm gaurantees to find the optimal merging
% choice by iteratively looping over all subgraph pairs and choose the one which has
% the maximum merging benefit.
% 
% INPUT: 
%   M: a square unsymmetric matrix that defines the target undirected graph to be partitioned.
%   Can be either the adjacency matrix or the edge-weights matrix of the
%   target graph. Edge-weights will not be considered if M is the dajacency
%   matrix.
%  
%   belonging_vector: a column vector whose values indicate which
%   subgraphs the vertices belong to. For example,"belonging_vector =
%   [1;2;2;1;3]" means the 1st subgraph = {1,4}, the 2nd subgraph = {2,3}
%   and the 3rd subgraph = {5}. 
%   
%   subgraph_info: information of all the subgraphs, such as vertices, number
%   of computation levels
% 
% 

    n_vertices = length(M);

    % get maximum allowed computation levels
    if nargin == 3
        % set to half the number of the vertices
        max_num_CLs = round(n_vertices/2);
    end

    n_graphs = length(subgraphs_info); % number of graphs
%     subgraph_IDs = 1:n_graphs;
%     
%     subgraph_sizes = histcounts(belonging_vector,'BinMethod','integers');

    % a matrix to record the benefit of merging two subgraphs, i.e., the sum of edge-weights between two merging subgraphs 
    benefit_matrix = zeros(n_graphs,n_graphs);

    % record the number of computation levels after merging two subgraphs
    num_CLs_matrix = zeros(n_graphs,n_graphs); 

    % whether a merging happened
    is_merging_happened = true;

    % record the subgraphs whose merging results with other subgraphs have
    % been checked
    is_checked = false(1,n_graphs);

    while is_merging_happened
        is_merging_happened = false;

        for subgraph_i = 1:(n_graphs-1)

            if ~is_checked(subgraph_i)
                % if the selected subgraph has not been checked yet
                is_checked(subgraph_i) = true;

                vertices_of_i = subgraphs_info(subgraph_i).vertices;

                for subgraph_j = (subgraph_i+1):n_graphs % no repeated check
                    
                    vertices_of_j = subgraphs_info(subgraph_j).vertices;
    
                    vertices_to_be_merged = [vertices_of_i,vertices_of_j];
                    M_merged = M(vertices_to_be_merged,vertices_to_be_merged);
                    M_diagonal = blkdiag(M(vertices_of_i,vertices_of_i), M(vertices_of_j,vertices_of_j));
                    % The matrix `M_merged` has at least as many nonzero entries compared to `M_diagonal`. 
                    % The non-zero entries in `M_merged` where they are zeros in `M_diagonal` are because of the edges
                    % connecting the two selected subgraphs.

                    % First calculate benefit (fast) then check mergeability (slow) to save computation time
                    benefit = sum(M_merged-M_diagonal,'all'); % calculate benefit (fast)

                    if benefit > 0
                        [isvalid, L] = kahn(M_merged); % check mergeability (slow)
                        if isvalid && size(L,1)<=max_num_CLs
                            num_CLs_matrix(subgraph_i,subgraph_j) = size(L,1);
                            benefit_matrix(subgraph_i,subgraph_j) = benefit;
                        end
                    end
                end
            end
        end

        % compare the benefits
        [max_benefit, idx] = max(benefit_matrix,[],'all','linear');
        
        % merging is only meaningful if benefit is greater than zero
        if max_benefit > 0
            % find the two subgraphs to be merged
            [subgraph_higher_id, subgraph_smaller_id] = ind2sub(size(benefit_matrix), idx);
            
            vertices_lower_id = subgraphs_info(subgraph_smaller_id).vertices;
            % merge the subgraoh with a smaller ID to the subgraph with a higher ID
            subgraphs_info(subgraph_higher_id).vertices = [subgraphs_info(subgraph_higher_id).vertices, vertices_lower_id];
            subgraphs_info(subgraph_higher_id).num_CLs = num_CLs_matrix(subgraph_higher_id,subgraph_smaller_id);

            % update the belonging vector
            belonging_vector(vertices_lower_id) = subgraph_higher_id;

            % set the checking status of the new subgraph to false so that
            % it will be check in the next iteration
            is_checked(subgraph_higher_id) = false;

            % delete the corresponding elements of the merged subgraph with a smaller ID
            subgraphs_info(subgraph_smaller_id) = [];
            is_checked(subgraph_smaller_id) = [];

            benefit_matrix(subgraph_smaller_id,:) = [];
            benefit_matrix(:,subgraph_smaller_id) = []; 

            % reset the merging benefits with the new subgraph 
            benefit_matrix(subgraph_higher_id,:) = 0;
            benefit_matrix(:,subgraph_higher_id) = 0;

            is_merging_happened = true;
        end

        n_graphs = length(subgraphs_info);
    end

    % update the belonging vector
    for i = 1:length(subgraphs_info)
        belonging_vector(subgraphs_info(i).vertices) = i;
    end

%     graphs_visualization(belonging_vector, M, 'ShowWeights', true)
end
