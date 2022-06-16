function [parl_groups, subgraphs_info, belonging_vector] = form_parallel_groups(M, varargin)
% FORM_PARALLEL_GROUPS Form parallel graoups of vehicles based on the given
% matrix, which can either be a directed adjacency matrix or a edge-weights
% matrix, while ensuring that the number of computation levels of each
% group do not exceed a number defined by the second input argument. 
    
    % Process optional input and Name-Value pair options
    [M, max_num_CLs, method] = parse_inputs(M, varargin{:});

    % partition the supergraph to subgraphs with a certain upper graph size.
    % The sum of weights of edges connecting subgraphs is the objective value and should be minimized.
    [belonging_vector, subgraphs_info] = graph_partitioning_algorithm(M, max_num_CLs, 'method', method);
    
    % subgraphs is mergeable if the number of computation levels of the
    % new graph does not exceed the maximum allowed number.
    [belonging_vector, subgraphs_info] = graph_merging_algorithm(M, belonging_vector, subgraphs_info, max_num_CLs);

%     graphs_visualization(belonging_vector, M, 'ShowWeights', true)
    
    n_grps = length(subgraphs_info); % one subgraph corresponds to one parallel group
    directed_adjacency = (M ~= 0);

    CLs_max_grps = max([subgraphs_info.num_CLs]); % max number of computation levels among all parallel groups
    parl_groups(CLs_max_grps) = struct('members',[],'predecessors',[]); % gather vehicles that are in the same computation level 
    
    % form the parallel groups
    for grp_i = 1:n_grps

        vertices_in_i = subgraphs_info(grp_i).vertices; % vertices in the group_i
        directed_adjacency_i = directed_adjacency(vertices_in_i,vertices_in_i);
        [valid,L] = kahn(directed_adjacency_i);
        assert(valid==true)
        subgraphs_info(grp_i).L_topology = L;

        num_CLs = size(L,1);
        for level_i = 1:num_CLs
            
            members_local = (L(level_i,:) == 1);
            members = vertices_in_i(members_local);
            parl_groups(level_i).members = [parl_groups(level_i).members, members];

            % add vehicles at the current computation level as predecessors of all the following level 
            for level_j = level_i+1:CLs_max_grps
                parl_groups(level_j).predecessors = [parl_groups(level_j).predecessors, members];
            end
        end
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

