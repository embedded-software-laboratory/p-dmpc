function [belonging_vector, cost] = min_cut_MILP(M, varargin)
% MIN_CUT_MILP This program formulates the min cut clustering problem for a
% graph defined by the given matrix. The problem is solved via the MATLAB
% optimization toolbox `intlinprog` (mixed-interger linear programming). 
% 
% INPUT:
%   M: an m-by-m symmetric matrix, can be either adjacency matrix or
%   edge-weights matrix
%   
%   must_not_in_same_subset (optional input): cell array defines vertices that are not
%   allowed be put in the same subset. For example, "not_in_same_subset =
%   {[1,2,3],[2,8]}" means vertices 1, 2 and 3 are not allowed to be in one
%   subset and vertices 2 and 8 are also not allowed to be in the same
%   subset.  
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
%   cost: objective value to be minimized. The number of cut edges if M
%   is adjacency matrix; Or the sum of cut edge-weights if M is
%   edge-weights matrix. 
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
%   [belonging_vector_1, cost_1] = min_cut_MILP(M)
% 
%   % Next, add additional constraint that vertices {1,2,4} and {1,3,4} must not be in
%   % the same subset
%   must_not_in_same_subset = {[1,2,4],[1,3,4]};
%   [belonging_vector_2, cost_2] = min_cut_MILP(M, must_not_in_same_subset)
% 
%   % If vertices {1,2} must be in the same subset:
%   must_in_same_subset = [1,2];
%   [belonging_vector_3, cost_3] = min_cut_MILP(M, must_not_in_same_subset, must_in_same_subset)
     

    % Process optional input and Name-Value pair options
    [M, must_not_in_same_subset, must_in_same_subset] = parse_inputs(M, varargin{:});

    % must be symmetrix matrix
    if ~issymmetric(M)
        M = M + M';
    end
    
    assert(issymmetric(M)==true) 

    n_vertices = size(M,1);

    % number of sets of vertices that should not in the same subset 
    n_not_in_same_subset = length(must_not_in_same_subset); 

    if ~isempty(must_in_same_subset)
        n_in_same_subset = 1;
    else
        n_in_same_subset = 0;
    end

    n_edges = (n_vertices.*(n_vertices-1))./2; 
    n_ineq = 4*n_edges + 2*n_not_in_same_subset + 2*n_in_same_subset; % max number of linear inequality constraints
    n_decision_vars = n_edges + n_vertices; % number of decision variables

    % index variables
    ineq_idx = 0; % inequality index
    edge_idx = 0; % edge index

    % initialize variables for mixed-integer linear programming (MILP)
    A = zeros(n_ineq,n_decision_vars);
    f = zeros(1,n_decision_vars);
    b = zeros(n_ineq,1);

    for i = 1:n_vertices-1
        for j = i+1:n_vertices
            edge_idx = edge_idx + 1;
            if M(i,j) ~= 0
                % objective value
                f(n_vertices+edge_idx) = M(i,j);
                % force edge(i,j) not being cut if vertex_i and vertex_j are in the same subset
                % inequality 1
                ineq_idx = ineq_idx + 1;
                A(ineq_idx,i) = -1;
                A(ineq_idx,j) = -1;
                A(ineq_idx,n_vertices+edge_idx) = 1;
                b(ineq_idx) = 0;
                % inequality 2
                ineq_idx = ineq_idx + 1;
                A(ineq_idx,i) = 1;
                A(ineq_idx,j) = 1;
                A(ineq_idx,n_vertices+edge_idx) = 1;
                b(ineq_idx) = 2;

                % force edge(i,j) being cut if vertex_i and vertex_j are in different subsets
                % inequality 3
                ineq_idx = ineq_idx + 1;
                A(ineq_idx,i) = 1;
                A(ineq_idx,j) = -1;
                A(ineq_idx,n_vertices+edge_idx) = -1;
                b(ineq_idx) = 0;
                % inequality 4
                ineq_idx = ineq_idx + 1;
                A(ineq_idx,i) = -1;
                A(ineq_idx,j) = 1;
                A(ineq_idx,n_vertices+edge_idx) = -1;
                b(ineq_idx) = 0;
            end
        end
    end
    
    % force the given sets of vertices not in the same subset
    for j = 1:n_not_in_same_subset
        not_in_same_subset_j = must_not_in_same_subset{j};
        
        ineq_idx = ineq_idx + 1;
        A(ineq_idx,not_in_same_subset_j) = 1;
        b(ineq_idx) = length(not_in_same_subset_j) - 1; % at least one vertex in this set should be separated from others
        
        ineq_idx = ineq_idx + 1;
        A(ineq_idx,not_in_same_subset_j) = -1;
        b(ineq_idx) = -1; % at least one vertex should not be separated
    end

    % force the given set of vertices in the same subset
    if ~isempty(must_in_same_subset)        
        ineq_idx = ineq_idx + 1;
        A(ineq_idx,must_in_same_subset) = 1;
        b(ineq_idx) = length(must_in_same_subset);
        
        ineq_idx = ineq_idx + 1;
        A(ineq_idx,must_in_same_subset) = -1;
        b(ineq_idx) = -length(must_in_same_subset);
    end

    % only the first cons's rows are relevant
    A = A(1:ineq_idx,:);
    b = b(1:ineq_idx);

    intcon = 1:n_decision_vars; % dicision variables that must be integers

    % define lower and upper bounds
    lb = zeros(1,n_decision_vars); 
    ub = ones(1,n_decision_vars); 
    
    % call solver: MILP
    [x, cost] = intlinprog(f,intcon,A,b,[],[],lb,ub);

%     V1 = find(x(1:m)>0.5); % vertices belong to the first subset for all entries in x equal to one
%     V2 = setdiff(1:m,V1)'; % vertices belong to the second subset for all entries in x equal to zeros
    
    if ~isempty(x)
        belonging_vector = round(x(1:n_vertices)); % turn 1.000 to 1
    else
        % if the solver is unable to find a feasible solution
        belonging_vector = [];
        warning('No feasible solution can be found for mixed-integer linear programming.')
    end

end


%% local function: parse_inputs
function [M, must_not_in_same_subset, must_in_same_subset] = parse_inputs(M, varargin)
    % Process optional input and Name-Value pair options
    
    must_not_in_same_subset_default = {1:length(M)}; % at least one vertex should be cut (to avoid empty cut)
    must_in_same_subset_default = []; % default empty

    p = inputParser;
    addRequired(p,'M', @(x) isnumeric(x) && ismatrix(x)); % must be numerical matrix
    addOptional(p,'must_not_in_same_subset', must_not_in_same_subset_default, @(x) isempty(x) || (iscell(x))); % must be cell or empty
    addOptional(p,'must_in_same_subset', must_in_same_subset_default, @(x) isempty(x) || (isnumeric(x) && isvector(x))); % must be numerical vector or empty
    parse(p, M, varargin{:}); % start parsing
    
    % get parsed inputs
    M = p.Results.M;
    must_not_in_same_subset = p.Results.must_not_in_same_subset;
    must_in_same_subset = p.Results.must_in_same_subset;

end