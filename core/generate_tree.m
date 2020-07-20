% generates search tree of all feasible trajectories
% init_state refers to the initial state of our vehicle [x_init, y_init, yaw_init]
% target specifies the position and angle of 
% trim refers to the list of trim tuples
% maneuver refers to the matrix of maneuvers
% depth specifies the depth of the created search tree
% dt specifies the time of our 
function search_tree = generate_tree(init_state, target, trim, maneuver, search_depth, dt)
    

    %% Import tree class
    % matlab_tree = '../matlab-tree/';
    % assert(logical(exist(matlab_tree, 'dir')));
    % addpath(matlab_tree);

    %% Initialize tree
    cur_state = init_state;
    cur_node = node(1, 0, 0, trim, cur_state);
    
    search_tree = tree(cur_node);
    
    next_index = 1; 
    %% expand search tree until target or trim is reached (TODO)
    while depth(search_tree) < search_depth
        
        next_state.x = cur_state.x + maneuver{1, 1}.dx;  
        next_state.y = cur_state.y + maneuver{1, 1}.dy;  
        next_state.yaw = cur_state.yaw + maneuver{1, 1}.dyaw;  
        next_node = node(2, 1, 0, trim, next_state);
        cur_state = next_state;
        
        [search_tree, next_index] = search_tree.addnode(next_index, next_node);
    end
    
    disp(search_tree.tostring)
end

