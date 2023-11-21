function directed_coupling_sequential = form_parallel_groups(M, coupling_info, max_num_CLs, method)
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
    %   target graph. Edge-weights will not be considered if M is the adjacency
    %   matrix.
    %
    %   coupling_info: coupling information of each coupling pair
    %
    %   max_num_CLs: maximum number of computation levels
    %
    %   method: either 's-t-cut' or 'MILP'
    %
    % OUTPUT:
    %
    %   subgraphs_info: information of all the subgraphs, such as vertices, number
    %   of computation levels
    directed_coupling_sequential = zeros(size(M));

    if max_num_CLs == 1
        % No sequential couplings
        return
    end

    % partition the given graph to subgraphs with a certain upper graph size.
    % The sum of weights of edges connecting subgraphs is the objective value and should be minimized.
    [belonging_vector, subgraphs_info] = graph_partitioning_algorithm(M, coupling_info, max_num_CLs, method);

    % subgraphs is mergeable if the number of computation levels of the
    % new graph does not exceed the maximum allowed number.
    subgraphs_info = graph_merging_algorithm(M, belonging_vector, subgraphs_info, max_num_CLs);

    directed_coupling_sequential(M ~= 0) = 1;

    n_grps = length(subgraphs_info); % one subgraph corresponds to one parallel group

    for i_graph = 1:n_grps
        parallel_vertices = setdiff(1:length(M), subgraphs_info(i_graph).vertices);
        directed_coupling_sequential(subgraphs_info(i_graph).vertices, parallel_vertices) = 0;
        directed_coupling_sequential(parallel_vertices, subgraphs_info(i_graph).vertices) = 0;
    end

end
