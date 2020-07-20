% Generates search tree of all feasible trajectories
% init_state refers to the initial state of our vehicle [x_init, y_init, yaw_init]
% target specifies the position and angle of 
% trim refers to the list of trim tuples
% maneuver refers to the matrix of maneuvers
% depth specifies the depth of the created search tree
% dt specifies the time of our 
function search_graph = generate_tree(init_state, target, trim, maneuver, search_depth, dt)
    
    id =  1
    value = 0;
    x = init_state.x
    y = init_state.y
    yaw = init_state.yaw
    
    search_graph = digraph;
    node = table(id, 0, trim, x, y, yaw)
    search_graph = addnode(search_graph, node);
    
    % Expand graph until depth or target is reached (TODO)
    while length(dfsearch(search_graph,1)) < search_depth
        node = table(id, 0, trim, x, y, yaw)
        search_graph = addnode(search_graph, node);
        search_graph = addedge(search_graph, id, id + 1);
        id = id + 1;
    end
    
    h = plot(search_graph);
end

