function [directed_coupling_sequential, subgraphs_info, belonging_vector] = form_parallel_groups(M, coupling_info, max_num_CLs, is_force_parallel_vehs_in_same_grp, method)
    % TODO Substitute with cut method.
    % function result = sequential_coupling(weighted_coupling)

    % FORM_PARALLEL_GROUPS Form parallel groups of vehicles based on the given
    % matrix M, which can either be a directed adjacency matrix or a weighting
    % matrix, while ensuring that the number of computation levels of each
    % group do not exceed a value `max_num_CLs`.
    %
    % INPUT:
    %   M: a square unsymmetric matrix that defines the target undirected graph to be partitioned.
    %   Can be either the adjacency matrix or the edge-weights matrix of the
    %   target graph. Edge-weights will not be considered if M is the dajacency
    %   matrix.
    %
    %   coupling_info: coupling information of each coupling pair
    %
    %   max_num_CLs: maximum number of computation levels
    %
    %   is_force_parallel_vehs_in_same_grp:
    %       boolean whether to force parallel driving vehicles
    %       being in the same group
    %
    %   method: either 's-t-cut' or 'MILP'
    %
    % OUTPUT:
    %
    %   subgraphs_info: information of all the subgraphs, such as vertices, number
    %   of computation levels
    %
    %   belonging_vector: a column vector whose values indicate which
    %   subgraphs the vertices belong to. For example,"belonging_vector =
    %   [1;2;2;1;3]" means the 1st subgraph = {1,4}, the 2nd subgraph = {2,3}
    %   and the 3rd subgraph = {5}.
    directed_coupling_sequential = zeros(size(M));

    if max_num_CLs == 1
        % pure parallel trajectory planning
        nVeh = length(M);

        belonging_vector = 1:nVeh;
        subgraphs_info(nVeh) = struct('vertices', [], 'num_CLs', [], 'path_info', []);

        for i = 1:nVeh
            subgraphs_info(i).vertices = i;
        end

        return
    end

    directed_coupling_sequential(M ~= 0) = 1;
    % partition the given graph to subgraphs with a certain upper graph size.
    % The sum of weights of edges connecting subgraphs is the objective value and should be minimized.
    [belonging_vector, subgraphs_info] = graph_partitioning_algorithm(M, coupling_info, max_num_CLs, is_force_parallel_vehs_in_same_grp, method);

    % subgraphs is mergeable if the number of computation levels of the
    % new graph does not exceed the maximum allowed number.
    [belonging_vector, subgraphs_info] = graph_merging_algorithm(M, belonging_vector, subgraphs_info, max_num_CLs);

    %     plot_partitioned_graph(belonging_vector, M, 'ShowWeights', true)

    n_grps = length(subgraphs_info); % one subgraph corresponds to one parallel group
    directed_coupling = (M ~= 0);

    CLs_max_grps = max([subgraphs_info.num_CLs]); % max number of computation levels among all parallel groups

    % form the parallel groups
    for grp_i = 1:n_grps

        vertices_in_i = subgraphs_info(grp_i).vertices; % vertices in the group_i
        directed_coupling_i = directed_coupling(vertices_in_i, vertices_in_i);
        L = kahn(directed_coupling_i);
        subgraphs_info(grp_i).L_topology = L;

    end

    for i_graph = 1:length(subgraphs_info)
        other_vertices = setdiff(1:length(M), subgraphs_info(i_graph).vertices);
        directed_coupling_sequential(subgraphs_info(i_graph).vertices, other_vertices) = 0;
        directed_coupling_sequential(other_vertices, subgraphs_info(i_graph).vertices) = 0;
    end

end
