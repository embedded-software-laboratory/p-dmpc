function [belonging_vector, cost, cutting_info] = min_cut_s_t(M, varargin)
% MIN_CUT_S_T Cut the given edge-weighted graph into two
% parts while ensuring the sum of the weights of the edges being cut is
% minimum.
% 
% INPUT:
%   M: an m-by-m symmetric matrix, can be either adjacency matrix or
%   edge-weights matrix
%   
%   must_not_in_same_subset (optional input): cell array defines vertices
%   that are not allowed be put in the same subset. For example,
%   "not_in_same_subset = {[1,2,3],[2,8]}" means vertices 1, 2 and 3 are
%   not allowed to be in one subset and vertices 2 and 8 are also not
%   allowed to be in the same subset. NOTE this program do not guarantee
%   all those constraints can be satisfied.
% 
%   must_in_same_subset (optional input): a vector defines vertices that must be
%   in the same subset. For example, "in_same_subset = [1,2,3]" means
%   vertices 1, 2 and 3 must not be separated. Note that multiple sets of
%   vertices that must be in the same subset are not supported by this program.
% 
% OUTPUT:
%   belonging_vector: a column vector whose values indicate which
%   subgraph a vertex belongs to. For example,"belonging_vector =
%   [1;2;2;1;3]" means the following three subgraphs: {1,4}, {2,3} and {5}.
% 
%   cost: the sum of the edges of weights being cut. Objective value to be
%   minimized.
% 
%   cutting_info: informations of all minimum cut phases.
% 
% COMPLEXITY:
%   O(|E|+|V|*log|V|)
% 
% REFERENCE:
%   Stoer and Wagner. A Simple Min Cut Algorithm. 1997
% 
% EXAMPLE:
%   M = [0, .2, .1,  0;
%        0,  0,  0, .5;
%        0,  0,  0, .1;
%        0,  0,  0,  0];
%   G = digraph(M);
%   plot(G, 'EdgeLabel', G.Edges.Weight) % visualize the graph
% 
%   % Without additional constraints 
%   % Since vertices {1,2,4} are in the same subset after cutting, the results are therefore unsatisfied if the maximum computation levels is 2.
%   [belonging_vector_1, cost_1] = min_cut_s_t(M)
% 
%   % Next, add additional constraint that vertices {1,2,4} and {1,3,4} must not be in
%   % the same subset
%   must_not_in_same_subset = {[1,2,4],[1,3,4]};
%   [belonging_vector_2, cost_2] = min_cut_s_t(M, must_not_in_same_subset)
% 
%   % If vertices {1,2} must be in the same subset:
%   must_in_same_subset = [1,2];
%   [belonging_vector_3, cost_3] = min_cut_s_t(M, must_not_in_same_subset, must_in_same_subset)
    
    % Process optional input and Name-Value pair options
    [M, must_not_in_same_subset, must_in_same_subset] = parse_inputs(M, varargin{:});

    % number of vertices
    nVertices = length(M);    
    
    % remove self edges and inf-value
    M(1:nVertices+1:end) = 0;   
    M(isinf(M)) = 0; 

    % turn to symmetric if not
    if ~issymmetric(M)
        M = M + M';
    end


    % reduced edge-weights matrix after merging some vertices
    M_reduced = M;

    % predefined variables
    belonging_vector = (1:nVertices)';

    % record the cutting information
    cutting_info = struct('belonging_vector',[],'num_valid_cut_paths',[],'cost',[]);

    vertex_indices = 1:nVertices;
    % reduced vertex indices after merging some vertices
    vertex_indices_reduced = vertex_indices;

    % make vertices inseparable if required
    if length(must_in_same_subset)>=2
        % merge the vertices which should not be separated, so they'll be unbreakable
        must_in_same_subset = sort(must_in_same_subset);
        merging_point = must_in_same_subset(1);

        for i = length(must_in_same_subset):-1:2
            vertes_to_be_merged = must_in_same_subset(i);
            [M_reduced, vertex_indices_reduced] = merge_two_vertices(M_reduced, merging_point, vertes_to_be_merged, vertex_indices_reduced);
        end
    
        % update belonging vector
        belonging_vector(must_in_same_subset) = must_in_same_subset(1);
    end

    count = 1;
    while length(vertex_indices_reduced)>1
        % vertex-s and -t are the last two vertices being merged during the minimum cut phase
        [vertex_s_local,vertex_t_local] = minimum_cut_phase(M_reduced); 
        
        % real indices of vertex-s and -t
        vertex_s = vertex_indices_reduced(vertex_s_local);
        vertex_t = vertex_indices_reduced(vertex_t_local);

        cost_tmp = sum(M_reduced(vertex_t_local,:));

        % in case vertex_t is a merging point
        vertices_in_point_t = find(belonging_vector==vertex_t)';

        % store cost
        cutting_info(count).cost = cost_tmp;
        % calculate the number of valid cut paths, which equals to the
        % number of cut paths that must be cut (defined by `must_not_in_same_subset`) 
        check_if_in_part_t = cellfun(@(c) sum(ismember(c,vertices_in_point_t))~=0, must_not_in_same_subset, 'UniformOutput', false);
        check_if_fully_in_part_t = cellfun(@(c) sum(ismember(c,vertices_in_point_t))==length(c), must_not_in_same_subset, 'UniformOutput', false);
         
        num_valid_cut_paths = nnz(cell2mat(check_if_in_part_t)) - nnz(cell2mat(check_if_fully_in_part_t));
        cutting_info(count).num_valid_cut_paths = num_valid_cut_paths;

        belong_vector_tmp = belonging_vector;
        belong_vector_tmp(belonging_vector==belonging_vector(vertex_t)) = 0; % zeros for one part
        belong_vector_tmp(belong_vector_tmp>0) = 1; % ones for another part
        cutting_info(count).belonging_vector = belong_vector_tmp;
        
        % merge vertex_t into vertex_s
        [M_reduced, vertex_indices_reduced] = merge_two_vertices(M_reduced, vertex_s_local, vertex_t_local, vertex_indices_reduced);

        % update belonging vector: set vertex_t in the belonging_vector to vertex_s
        belonging_vector(belonging_vector == belonging_vector(vertex_t)) = belonging_vector(vertex_s);

        count = count + 1;
    end

    valid_minimum_cut_phases_idx = [];
    n_must_not_in_same_subset = length(must_not_in_same_subset);
    % Find which minimum cut phases satisfy the additional constraints.
    % If no one satisfy all the constraints, iteratively reduce the number of
    % constraints by one and find again.
    while isempty(valid_minimum_cut_phases_idx) && n_must_not_in_same_subset>=1
        valid_minimum_cut_phases_idx = find([cutting_info.num_valid_cut_paths] >= n_must_not_in_same_subset);
        n_must_not_in_same_subset = n_must_not_in_same_subset - 1;
    end

    if isempty(valid_minimum_cut_phases_idx)
        % if no minimum cut phase has a valid cut path, choose the
        % one with the lowest cost among all of the minimum cut phases
        valid_minimum_cut_phases_idx = 1:length(cutting_info);
    end

    % find all minimum cut pahses that has at least one valid cut path
    % and choose the one with the lowest cost among them
    [cost, lowest_cost_idx_local] = min([cutting_info(valid_minimum_cut_phases_idx).cost]);
    lowest_cost_idx = valid_minimum_cut_phases_idx(lowest_cost_idx_local);
    belonging_vector = cutting_info(lowest_cost_idx).belonging_vector;
end


%% local function: merge_two_vertices
function [edge_weights_reduced, updated_vertex_indices] = merge_two_vertices(edge_weights, vertex_1, vertex_2, vertex_indices)
    % merge vertex_2 into vertex_1, which includes merging all the incoming
    % and outcoming edges of vertex_1 into vertex_2

    % weight change
    edge_weights(vertex_1,:) = edge_weights(vertex_1,:) + edge_weights(vertex_2,:);
    edge_weights(:,vertex_1) = edge_weights(vertex_1,:);
    edge_weights(vertex_1,vertex_1)= 0; % set self-edge to zero
    
    % delete vertex 1 and keep remaining vertices
    select_remaining = true(1, length(edge_weights));
    select_remaining(vertex_2) = false;
    edge_weights_reduced  = edge_weights(select_remaining, select_remaining);
    updated_vertex_indices = vertex_indices(select_remaining);
end


%% local function: minimum_cut_phase
function [vertex_s, vertex_t] = minimum_cut_phase(edge_weights)
% First initialize the merging point with a single (and randomly) selected
% starting vertex. 
% Next, find the most tightly connected vertex (vertex that has the
% hightest edge-weight connected to it) with the merging point and merging
% the found vertex into it. After doing this nVertices-1 times, all
% vertices would be merged into the merging point. The second last merged
% vertex-s and the last merged vertex-t are esprecially important and will
% be returned. 

    nVertices  = length(edge_weights);

    % special case: if totally only two vertices
    if nVertices == 2                   
        vertex_s = 1;     
        vertex_t = 2;     
        return;
    end

    % Choose the starting vertex. 
    % Note that one can choose an arbitrary vertex as the starting vertex.
    % The result does not depends on the choosing of the starting vertex
    % except one case: there are more than one way to cut the graph with
    % exactly the same lowest cost.
    % To exclude this case, here always choose the first vertex as the
    % starting vertex while not impacting the performance. 
    starting_vertex = 1; % also randomly choose: starting_vertex = randi(nVertices);
    
    % indices of all merged vertices
    merged_vertices = starting_vertex;

    % sum of edge-weights between each vertex to the merging point. Zero
    % for all vertices inside the merging point.
    sum_weights_to_merging_point = edge_weights(starting_vertex,:);
    % store the verties that are waiting for merging in a queue according
    % to their sum of edge-weights connected to the merging point. The one
    % with the higest sum of weights will be merged first.
    [~,queue]= sort(sum_weights_to_merging_point,'descend');
    % delete the already merged vertices in the queue
    queue(ismember(queue,merged_vertices)) = []; 

    % Do merging nVertices-3 times until all vertices except two last
    % vertices are merged.
    % Note that one can also do it nVertices-1 times to complete the
    % merging process. But to save computation time the last two merging
    % step can be saved.
    for n = 1:nVertices-3
        % find the most tightly connected vertex (vertex that has the
        % hightest edge-weight connected to the starting vertex) with the
        % starting vertex and merge it into the starting vertex 
        most_tighly_connected_vertex = queue(1); 
        % update the merged vertices
        merged_vertices = [merged_vertices,most_tighly_connected_vertex];
        % update the sum of weights
        sum_weights_to_merging_point = sum_weights_to_merging_point + edge_weights(most_tighly_connected_vertex,:);
        % no edge between merged vertices
        sum_weights_to_merging_point(merged_vertices) = 0;
        [~,queue]= sort(sum_weights_to_merging_point,'descend');
        % delete the already merged vertices in the queue
        queue(ismember(queue,merged_vertices)) = [];
    end

    % Now only two vertices left in the queue. The first one is the
    % vertex-s, the second one is the vertex-t.
    vertex_s = queue(1);
    vertex_t = queue(2);
end
    
%% local function: parse_inputs
function [M, must_not_in_same_subset, must_in_same_subset] = parse_inputs(M, varargin)
    % Process optional input and Name-Value pair options
    
    must_not_in_same_subset_default = {1:length(M)}; % at least one vertex should be cut (to avoid empty cut)
    must_in_same_subset_default = []; % default empty

    p = inputParser;
    addRequired(p,'M', @(x) isnumeric(x) && ismatrix(x)); % must be numerical matrix
    addOptional(p,'must_not_in_same_subset', must_not_in_same_subset_default, @(x) iscell(x)); % must be cell or empty
    addOptional(p,'must_in_same_subset', must_in_same_subset_default, @(x) isempty(x) || (isnumeric(x) && isvector(x))); % must be numerical vector or empty
    parse(p, M, varargin{:}); % start parsing
    
    % get parsed inputs
    M = p.Results.M;
    must_not_in_same_subset = p.Results.must_not_in_same_subset;
    must_in_same_subset = p.Results.must_in_same_subset;

end