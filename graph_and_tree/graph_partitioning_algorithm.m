function [belonging_vector, subgraphs_infos] = graph_partitioning_algorithm(M, varargin)
% GRAPH_PARTITIONING_ALGORITHM Partition the given edge-weighted directed acyclic graph (DAG) while ensuring
% the size of each subgraph do not exceed the defined maximum graph size.
% 
% INPUT:
%   M: a square unsymmetric matrix that defines the target undirected graph to be partitioned.
%   Can be either the adjacency matrix or the edge-weights matrix of the
%   target graph. Edge-weights will not be considered if M is the dajacency
%   matrix.
%   
%   max_num_CLs: maximum number of computation levels, equals to the height
%   of the directed tree for the given matrix.
%
% OUTPUT:
%   belonging_vector: a column vector whose values indicates which
%   subgraphs the vertices belong to. For example,"belonging_vector =
%   [1;2;2;1;3]" means the 1st subgraph = {1,4}, the 2nd subgraph = {2,3}
%   and the 3rd subgraph = {5}. 
% 
%   subgraph_infos: information of all the subgraphs, such as vertices, number
%   of computation levels
    
    % Process optional input and Name-Value pair options
    [M, max_num_CLs, method] = parse_inputs(M, varargin{:});

    G_undirected = digraph(M);
    assert(isdag(G_undirected)) % check whether DAG
    
    % decompose the supergraph if it contains unconnected components
    [belonging_vector,~] = conncomp(G_undirected,'Type','weak');

%     graphs_visualization(belonging_vector, M, 'ShowWeights', true)
    % number of graphs
    graph_indices = unique(belonging_vector); 
    n_graphs = length(graph_indices);

    subgraphs_infos(n_graphs) = struct('vertices',[],'num_CLs',[],'path_infos',[]);
    
%     L_topologies = {1,n_graphs};
    for i_graph=graph_indices
        vertices_i = find(belonging_vector==i_graph);
%         % Kahn's topological sorting algorithm
%         [valid, L_topologies{i_graph}] = kahn(M(vertices_i,vertices_i));
%         assert(valid==true);

        % get all paths from source vertices to sinks, since the computation levels are only depends on those paths 
        path_infos_i = get_longest_paths_from_sources_to_sinks(M(vertices_i,vertices_i));
        % store all paths from source vertices to sink vertices
        subgraphs_infos(i_graph).vertices = vertices_i;
        subgraphs_infos(i_graph).path_infos = path_infos_i;
        subgraphs_infos(i_graph).num_CLs = path_infos_i(1).length;
    end

    % get the current longest subgraph (subgraph who has the maximum number of computation levels)
    [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_infos.num_CLs]);
%     L_topologies = L_topologies(queue);

    % cut the longest graph into two parts until no graph is longer than defined
    while num_CLs_longest_graph>max_num_CLs
        vertices_longest_graph = find(belonging_vector==longest_graph_ID); % vertices that belong to the longest graph
        M_longest_graph = M(vertices_longest_graph,vertices_longest_graph);

        % constraints on which paths must be cut
        must_not_in_same_subset = {subgraphs_infos(longest_graph_ID).path_infos(1).path}; % cell array
        % constraint on which path must not be cut
        must_in_same_subset = []; % vector

        % call program to cut the longest graph to two parts
        switch method 
            case 's-t-cut'
                belonging_vector_longest_graph = min_cut_s_t(M_longest_graph, must_not_in_same_subset, must_in_same_subset);
            case 'MILP'
                belonging_vector_longest_graph = min_cut_MILP(M_longest_graph, must_not_in_same_subset, must_in_same_subset);
                if isempty(belonging_vector_longest_graph)
                    warning('Change to use minimum s-t-cut algorithm.')
                    belonging_vector_longest_graph = min_cut_s_t(M_longest_graph, must_not_in_same_subset, must_in_same_subset);
                end
            case 'auto'
                % to be finished if needed
            otherwise
                error('Invalid method name for graph cutting.')
        end
        
        % Cut the longest graph into two parts while ensuring the sum of the weights f the edges being cut is minimum 
        % Returns a vector contains only zeros and ones. Zeros for the first part, ones for the second part.
%         options.L_topology = L_topologies{1};
%         options.vertices_inseparable = [];
%         belonging_vector_longest_graph = min_cut_s_t(edge_weights_longest_graph,options);



%         belonging_vector_longest_graph = min_cut_s_t(M_longest_graph);
    
        vertices_first_part = vertices_longest_graph((belonging_vector_longest_graph==0));
        vertices_second_part = vertices_longest_graph((belonging_vector_longest_graph==1));
        % part which has more vertices is considered as the first part
        if length(vertices_second_part)>length(vertices_first_part)
            % swap two variables
            [vertices_first_part, vertices_second_part] = swap(vertices_first_part, vertices_second_part);
        end

        % number of graphs add 1 after cutting the longest subgraph into two parts
        n_graphs = n_graphs + 1;

        % keep the index of the first part, assign the second part with the highest graph index
        belonging_vector(vertices_second_part) = n_graphs;


        % calculate the number of computation levels for the two new subgraphs
%         [valid, L_topologies{1}] = kahn(M(indices_first_part,indices_first_part));
%         assert(valid==true)
%         number_CLs_first_part = size(L_first_part,1);
%         [valid, L_topologies{end+1}] = kahn(M(indices_second_part,indices_second_part));
%         assert(valid==true)
%         number_CLs_second_part = size(L_second_part,1);
        
        path_infos_first_part = get_longest_paths_from_sources_to_sinks(M(vertices_first_part,vertices_first_part)); 
        % replace the previous longest graph with the first part  
        subgraphs_infos(longest_graph_ID).vertices = vertices_first_part;
        subgraphs_infos(longest_graph_ID).path_infos = path_infos_first_part;
        subgraphs_infos(longest_graph_ID).num_CLs = path_infos_first_part(1).length;
        
        path_infos_second_part = get_longest_paths_from_sources_to_sinks(M(vertices_second_part,vertices_second_part));
        % add the second part to the end 
        subgraphs_infos(n_graphs).vertices = vertices_second_part;
        subgraphs_infos(n_graphs).path_infos = path_infos_second_part;
        subgraphs_infos(n_graphs).num_CLs = path_infos_second_part(1).length;
        
        % get the current longest subgraph (subgraph who has the maximum number of computation levels)
        [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_infos.num_CLs]);

        % visualize subgraphs
%         graphs_visualization(belonging_vector, M, 'ShowWeights', true)

    end

end


%% local function
function [M, max_num_CLs, method] = parse_inputs(M, varargin)
    % Process optional input and Name-Value pair options
    
    n = size(M,1); % number of vertices
    % set the default value of the maximum number of computation levels to be half of the total number of vertices
    default_max_num_CLs = ceil(n/2); % round up

    default_method = 's-t-cut';
    expected_methods = {'auto', 's-t-cut', 'MILP'};

    p = inputParser;
    addRequired(p,'M',@(x) isnumeric(x) && ismatrix(x)); % must be numerical matrix
    addOptional(p,'max_num_CLs', default_max_num_CLs, @(x) isnumeric(x) && isscalar(x) && x>0); % must be numerical scalar
    addParameter(p,'method',default_method, @(x) any(validatestring(x,expected_methods)));
    parse(p, M, varargin{:}); % start parsing
    
    % get parsed inputs
    M = p.Results.M;
    max_num_CLs = p.Results.max_num_CLs;
    method = p.Results.method;

end

