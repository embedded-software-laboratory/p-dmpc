function [parl_groups, subgraphs_info, belonging_vector] = form_parallel_groups(M, max_num_CLs, coupling_info, method, options)
    % FORM_PARALLEL_GROUPS Form parallel graoups of vehicles based on the given
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
    %   max_num_CLs: maximum number of computation levels
    %
    %   coupling_info: couling information of each coupling pair
    %
    %   method: either 's-t-cut' or 'MILP'
    %
    %   options: instance of the class `OptionsMain`
    %
    % OUTPUT:
    %   parl_groups: also called CL_based_hierarchy, specifying the computation
    %   level of each vehicle
    %
    %   subgraphs_info: information of all the subgraphs, such as vertices, number
    %   of computation levels
    %
    %   belonging_vector: a column vector whose values indicate which
    %   subgraphs the vertices belong to. For example,"belonging_vector =
    %   [1;2;2;1;3]" means the 1st subgraph = {1,4}, the 2nd subgraph = {2,3}
    %   and the 3rd subgraph = {5}.

    if max_num_CLs == 1
        % pure parallel trajectory planning
        nVeh = length(M);

        belonging_vector = 1:nVeh;
        parl_groups = struct('members', [], 'predecessors', []);
        parl_groups.members = 1:nVeh;
        subgraphs_info(nVeh) = struct('vertices', [], 'num_CLs', [], 'path_info', []);

        for i = 1:nVeh
            subgraphs_info(i).vertices = i;
        end

        return
    end

    % partition the given graph to subgraphs with a certain upper graph size.
    % The sum of weights of edges connecting subgraphs is the objective value and should be minimized.
    [belonging_vector, subgraphs_info] = graph_partitioning_algorithm(M, max_num_CLs, coupling_info, method, options);

    % subgraphs is mergeable if the number of computation levels of the
    % new graph does not exceed the maximum allowed number.
    [belonging_vector, subgraphs_info] = graph_merging_algorithm(M, belonging_vector, subgraphs_info, max_num_CLs);

    %     graphs_visualization(belonging_vector, M, 'ShowWeights', true)

    n_grps = length(subgraphs_info); % one subgraph corresponds to one parallel group
    directed_adjacency = (M ~= 0);

    CLs_max_grps = max([subgraphs_info.num_CLs]); % max number of computation levels among all parallel groups
    parl_groups(CLs_max_grps) = struct('members', [], 'predecessors', []); % gather vehicles that are in the same computation level

    % form the parallel groups
    for grp_i = 1:n_grps

        vertices_in_i = subgraphs_info(grp_i).vertices; % vertices in the group_i
        directed_adjacency_i = directed_adjacency(vertices_in_i, vertices_in_i);
        [valid, L] = kahn(directed_adjacency_i);
        assert(valid == true)
        subgraphs_info(grp_i).L_topology = L;

        num_CLs = size(L, 1);

        for level_i = 1:num_CLs

            members_local = (L(level_i, :) == 1);
            members = vertices_in_i(members_local);
            parl_groups(level_i).members = [parl_groups(level_i).members, members];

            % add vehicles at the current computation level as predecessors of all the following level
            for level_j = level_i + 1:CLs_max_grps
                parl_groups(level_j).predecessors = [parl_groups(level_j).predecessors, members];
            end

        end

    end

end
