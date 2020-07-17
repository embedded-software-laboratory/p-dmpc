% generates search tree of all feasible trajectories
% init_state refers to the initial state of our vehicle [x_init, y_init, yaw_init]
% target specifies the position and angle of 
% trim refers to the list of trim tuples
% maneuver refers to the matrix of maneuvers
% depth specifies the depth of the created search tree
% dt specifies the time of our 
function search_tree = generate_tree(init_state, target, trim, maneuver, depth, dt)
    

    %% Import tree class
    matlab_tree = '../matlab-tree/';
    assert(logical(exist(matlab_tree, 'dir')));
    addpath(matlab_tree);

    %% Initialize tree
    
    % sample node
    sample_node = node(1, 0, 0, 0, [0, 0, 0]);
    
    search_tree = tree(sample_node);
    
    

    [search_tree node1] = search_tree.addnode(1, sample_node);
    
    %% expand search tree until target or trim is reached (TODO)
    for i = 1:2
        [search_tree node1] = search_tree.addnode(node1, sample_node);
    end
    
    disp(search_tree.tostring)
end

