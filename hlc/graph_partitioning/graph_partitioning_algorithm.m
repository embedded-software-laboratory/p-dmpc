function [belonging_vector, subgraphs_info] = graph_partitioning_algorithm(M, max_num_CLs, coupling_info, method, options)
    % GRAPH_PARTITIONING_ALGORITHM Partition the given edge-weighted directed acyclic graph (DAG) while ensuring
    % the size of each subgraph do not exceed the defined maximum graph size.
    %
    % INPUT:
    %   M: a square unsymmetric matrix that defines the target undirected graph to be partitioned.
    %   Can be either the adjacency matrix or the edge-weights matrix of the
    %   target graph. Edge-weights will not be considered if M is the dajacency
    %   matrix.
    %
    %   max_num_CLs: maximum number of computation levels
    %
    %   coupling_info: couling information of each coupling pair
    %
    %   method: either 's-t-cut' or 'MILP'
    %
    %   options: instance of the class `OptionsMain`
    %
    % OUTPUT:
    %   belonging_vector: a column vector whose values indicate which
    %   subgraphs the vertices belong to. For example,"belonging_vector =
    %   [1;2;2;1;3]" means the 1st subgraph = {1,4}, the 2nd subgraph = {2,3}
    %   and the 3rd subgraph = {5}.
    %
    %   subgraph_info: information of all the subgraphs, such as vertices, number
    %   of computation levels

    multiple_vehs_drive_parallel = {};

    if options.is_force_parallel_vehs_in_same_grp

        if ~isempty([coupling_info.veh_with_ROW])
            % find all vehicles that drive in parallel
            coupling_info = coupling_info([coupling_info.is_move_side_by_side]);
            % sort according to the STAC so that parallel pair with higher STAC will be considered earlier
            [~, order] = sort([coupling_info.STAC]);
            coupling_info = coupling_info(order);
            vehs_drive_parallel = [coupling_info.veh_with_ROW; coupling_info.veh_without_ROW]';
        else
            vehs_drive_parallel = [];
        end

        % Deal with multiple vehicles drive in parallel
        for iParl = 1:size(vehs_drive_parallel, 1)
            vehs_drive_parallel_i = vehs_drive_parallel(iParl, :);
            find_cell_idx = find(cellfun(@(c) any(ismember(vehs_drive_parallel_i, c)), multiple_vehs_drive_parallel));

            if isempty(find_cell_idx) && length(vehs_drive_parallel_i) <= max_num_CLs
                % Add to a new parallel dirving group
                multiple_vehs_drive_parallel{end + 1} = vehs_drive_parallel_i;
            else
                new_members = sort(unique([multiple_vehs_drive_parallel{find_cell_idx}, vehs_drive_parallel_i]), 'ascend');
                % Check if the allowed number computation levels is exceeded
                if length(new_members) <= max_num_CLs
                    % Add to the existing parallel driving group
                    if length(find_cell_idx) == 1
                        multiple_vehs_drive_parallel{find_cell_idx} = new_members;
                    else
                        find_cell_idx = sort(find_cell_idx, 'ascend');
                        multiple_vehs_drive_parallel{find_cell_idx(1)} = new_members;
                        % delete others
                        to_delete = false(1, length(multiple_vehs_drive_parallel));
                        to_delete(find_cell_idx(2:end)) = true;
                        multiple_vehs_drive_parallel = multiple_vehs_drive_parallel(~to_delete);
                    end

                end

            end

        end

    end

    G_directed = digraph(M);

    assert(isdag(G_directed)) % check whether DAG

    % decompose the supergraph if it contains unconnected components
    [belonging_vector, ~] = conncomp(G_directed, 'Type', 'weak');

    %     plot_partitioned_graph(belonging_vector, M, 'ShowWeights', true)
    % number of graphs
    graph_indices = unique(belonging_vector);
    n_graphs = length(graph_indices);

    subgraphs_info(n_graphs) = struct('vertices', [], 'num_CLs', [], 'path_info', []);

    for i_graph = graph_indices
        vertices_i = find(belonging_vector == i_graph);

        % get all paths from source vertices to sinks, since the computation levels are only depends on those paths
        path_info_i = get_longest_paths_from_sources_to_sinks(M(vertices_i, vertices_i));
        % store all paths from source vertices to sink vertices
        subgraphs_info(i_graph).vertices = vertices_i;
        subgraphs_info(i_graph).path_info = path_info_i;
        subgraphs_info(i_graph).num_CLs = path_info_i(1).length;
    end

    % get the current longest subgraph (subgraph who has the maximum number of computation levels)
    [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_info.num_CLs]);

    % cut the longest graph into two parts until no graph is longer than defined
    while num_CLs_longest_graph > max_num_CLs
        must_in_same_subset = {};
        vertices_longest_graph = find(belonging_vector == longest_graph_ID); % vertices that belong to the longest graph
        M_longest_graph = M(vertices_longest_graph, vertices_longest_graph);

        % do not separate vehicles driving in parallel
        find_parallel_vehs_group = find(cellfun(@(c) all(ismember(c, vertices_longest_graph)), multiple_vehs_drive_parallel));

        if ~isempty(find_parallel_vehs_group)
            must_in_same_subset_global = multiple_vehs_drive_parallel(find_parallel_vehs_group);

            for j = 1:length(must_in_same_subset_global)
                [~, local_idx] = ismember(must_in_same_subset_global{j}, vertices_longest_graph);
                must_in_same_subset{j} = local_idx;
            end

        end

        % constraints on which paths must be cut
        %         must_not_in_same_subset = {subgraphs_info(longest_graph_ID).path_info(1).path}; % cell array
        must_not_in_same_subset = {};
        % call program to cut the longest graph to two parts
        switch method
            case 's-t-cut'
                belonging_vector_longest_graph = min_cut_s_t(M_longest_graph, must_not_in_same_subset, must_in_same_subset);
            case 'MILP'
                assert(length(must_in_same_subset) <= 1)
                must_in_same_subset = must_in_same_subset{1}; % cell to vector
                belonging_vector_longest_graph = min_cut_MILP(M_longest_graph, must_not_in_same_subset, must_in_same_subset);

                if isempty(belonging_vector_longest_graph)
                    warning("No feasible cutting found when using MILP. Change to use minimum s-t-cut algorithm.")
                    belonging_vector_longest_graph = min_cut_s_t(M_longest_graph, must_not_in_same_subset, must_in_same_subset{:});
                end

            otherwise
                error("Invalid method name for graph cutting. Please specify either 's-t-cut' or 'MILP'.")
        end

        vertices_first_part = vertices_longest_graph((belonging_vector_longest_graph == 0));
        vertices_second_part = vertices_longest_graph((belonging_vector_longest_graph == 1));
        % part which has more vertices is considered as the first part
        if length(vertices_second_part) > length(vertices_first_part)
            % swap two variables
            [vertices_first_part, vertices_second_part] = swap(vertices_first_part, vertices_second_part);
        end

        % number of graphs add 1 after cutting the longest subgraph into two parts
        n_graphs = n_graphs + 1;

        % keep the index of the first part, assign the second part with the highest graph index
        belonging_vector(vertices_second_part) = n_graphs;

        path_info_first_part = get_longest_paths_from_sources_to_sinks(M(vertices_first_part, vertices_first_part));
        % replace the previous longest graph with the first part
        subgraphs_info(longest_graph_ID).vertices = vertices_first_part;
        subgraphs_info(longest_graph_ID).path_info = path_info_first_part;
        subgraphs_info(longest_graph_ID).num_CLs = path_info_first_part(1).length;

        path_info_second_part = get_longest_paths_from_sources_to_sinks(M(vertices_second_part, vertices_second_part));
        % add the second part to the end
        subgraphs_info(n_graphs).vertices = vertices_second_part;
        subgraphs_info(n_graphs).path_info = path_info_second_part;
        subgraphs_info(n_graphs).num_CLs = path_info_second_part(1).length;

        % get the current longest subgraph (subgraph who has the maximum number of computation levels)
        [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_info.num_CLs]);

        % visualize subgraphs
        %         plot_partitioned_graph(belonging_vector, M, 'ShowWeights', true)

    end

end
