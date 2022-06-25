function [belonging_vector, subgraphs_info] = graph_partitioning_algorithm(M, varargin)
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
%   coupling_info: couling information of each coupling pair
% 
%   method: either 's-t-cut' or 'MILP'
%
% OUTPUT:
%   belonging_vector: a column vector whose values indicate which
%   subgraphs the vertices belong to. For example,"belonging_vector =
%   [1;2;2;1;3]" means the 1st subgraph = {1,4}, the 2nd subgraph = {2,3}
%   and the 3rd subgraph = {5}. 
% 
%   subgraph_info: information of all the subgraphs, such as vertices, number
%   of computation levels
    
    % Process optional input and Name-Value pair options
    [M, max_num_CLs, coupling_info, method] = parse_inputs(M, varargin{:});

    if ~isempty([coupling_info.veh_with_ROW])
        % find all vehicles that drive in parallel 
        coupling_info = coupling_info([coupling_info.is_drive_parallel]); 
        vehs_drive_parallel = [coupling_info.veh_with_ROW,coupling_info.veh_without_ROW];
        vehs_drive_parallel = reshape(vehs_drive_parallel,[],2);
    end

    G_directed = digraph(M);
    assert(isdag(G_directed)) % check whether DAG
    
    % decompose the supergraph if it contains unconnected components
    [belonging_vector,~] = conncomp(G_directed,'Type','weak');

%     graphs_visualization(belonging_vector, M, 'ShowWeights', true)
    % number of graphs
    graph_indices = unique(belonging_vector); 
    n_graphs = length(graph_indices);

    subgraphs_info(n_graphs) = struct('vertices',[],'num_CLs',[],'path_info',[]);

    for i_graph = graph_indices
        vertices_i = find(belonging_vector==i_graph);

        % get all paths from source vertices to sinks, since the computation levels are only depends on those paths 
        path_info_i = get_longest_paths_from_sources_to_sinks(M(vertices_i,vertices_i));
        % store all paths from source vertices to sink vertices
        subgraphs_info(i_graph).vertices = vertices_i;
        subgraphs_info(i_graph).path_info = path_info_i;
        subgraphs_info(i_graph).num_CLs = path_info_i(1).length;
    end

    % get the current longest subgraph (subgraph who has the maximum number of computation levels)
    [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_info.num_CLs]);

    % cut the longest graph into two parts until no graph is longer than defined
    while num_CLs_longest_graph > max_num_CLs
        vertices_longest_graph = find(belonging_vector==longest_graph_ID); % vertices that belong to the longest graph
        M_longest_graph = M(vertices_longest_graph,vertices_longest_graph);
        % constraint on which path must not be cut
        must_in_same_subset = {}; % cell

        if max_num_CLs >= 2
            % do not separate vehicles driving in parallel
            for i_parl = 1:size(vehs_drive_parallel,1)
                vehs_parl = vehs_drive_parallel(i_parl,:);
                [~,vehs_parl_local] = ismember(vehs_parl,vertices_longest_graph);
                if nnz(vehs_parl_local)==2
                    % if both two vehicles driving in parallel are in the
                    % selected subgraph
                    if ~any(ismember(vehs_parl_local,[must_in_same_subset{:}]))
                        % not allow add repeated vehicles
                        must_in_same_subset(end+1) = {vehs_parl_local};
                        disp(['Vehicle ' num2str(vehs_parl(1)) ' and ' num2str(vehs_parl(2)) ' must plan trajectories in sequence because they drive in parallel.'])
                    else
                        warning('More than two vehicles are driving in parallel!')
                    end
                end
            end
        end

        % constraints on which paths must be cut
        must_not_in_same_subset = {subgraphs_info(longest_graph_ID).path_info(1).path}; % cell array

        % call program to cut the longest graph to two parts
        switch method 
            case 's-t-cut' 
                belonging_vector_longest_graph = min_cut_s_t(M_longest_graph, must_not_in_same_subset, must_in_same_subset);
            case 'MILP'
                assert(length(must_in_same_subset)<=1)
                belonging_vector_longest_graph = min_cut_MILP(M_longest_graph, must_not_in_same_subset, must_in_same_subset);
                if isempty(belonging_vector_longest_graph)
                    warning("No feasible cutting found when using MILP. Change to use minimum s-t-cut algorithm.")
                    belonging_vector_longest_graph = min_cut_s_t(M_longest_graph, must_not_in_same_subset, must_in_same_subset{:});
                end
            otherwise
                error("Invalid method name for graph cutting. Please specify either 's-t-cut' or 'MILP'.")
        end
    
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
        
        path_info_first_part = get_longest_paths_from_sources_to_sinks(M(vertices_first_part,vertices_first_part)); 
        % replace the previous longest graph with the first part  
        subgraphs_info(longest_graph_ID).vertices = vertices_first_part;
        subgraphs_info(longest_graph_ID).path_info = path_info_first_part;
        subgraphs_info(longest_graph_ID).num_CLs = path_info_first_part(1).length;
        
        path_info_second_part = get_longest_paths_from_sources_to_sinks(M(vertices_second_part,vertices_second_part));
        % add the second part to the end 
        subgraphs_info(n_graphs).vertices = vertices_second_part;
        subgraphs_info(n_graphs).path_info = path_info_second_part;
        subgraphs_info(n_graphs).num_CLs = path_info_second_part(1).length;
        
        % get the current longest subgraph (subgraph who has the maximum number of computation levels)
        [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_info.num_CLs]);

        % visualize subgraphs
%         graphs_visualization(belonging_vector, M, 'ShowWeights', true)

    end

end


%% local function
function [M, max_num_CLs, coupling_info, method] = parse_inputs(M, varargin)
    % Process optional input and Name-Value pair options
    
    n = size(M,1); % number of vertices
    % set the default value of the maximum number of computation levels to be half of the total number of vertices
    default_max_num_CLs = ceil(n/2); % round up
    default_coupling_info = {};

    default_method = 's-t-cut';
    expected_methods = {'s-t-cut', 'MILP'};

    p = inputParser;
    addRequired(p,'M',@(x) isnumeric(x) && ismatrix(x)); % must be numerical matrix
    addOptional(p,'max_num_CLs', default_max_num_CLs, @(x) (isnumeric(x) && x>0) || isempty(x) ); % must be numerical scalar
    addOptional(p,'coupling_info', default_coupling_info, @(x) isstruct(x) || isempty(x)); % must be numerical scalar
    addParameter(p,'method',default_method, @(x) any(validatestring(x,expected_methods)));
    parse(p, M, varargin{:}); % start parsing
    
    % get parsed inputs
    M = p.Results.M;
    max_num_CLs = p.Results.max_num_CLs;
    coupling_info = p.Results.coupling_info;
    method = p.Results.method;
end



