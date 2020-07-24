% Generates search tree of all feasible trajectories
% init_pose refers to the struct representing the initial pose of our vehicle
% target_pose refers to the struct representing the target pose of our vehicle
% trim refers to the index of the initial trim
% maneuvers refers to the matrix of maneuvers of a motion graph
% search_depth specifies the depth of the created search tree
function search_graph = generate_tree(init_pose, target_pose, trim_index, maneuvers, search_depth)
    
    % Initialize node table values
    id =  1;
    value = 0;
    trim = trim_index;
    x = init_pose.x;
    y = init_pose.y;
    yaw = init_pose.yaw;
    
    % Safe length of trim vector and parent index for node expansion
    trim_size = length(maneuvers(1,:));
    parent = 0;
    
    % Create digraph with root node
    search_graph = digraph;
    node = table(id, value, trim, x, y, yaw);
    search_graph = addnode(search_graph, node);
    
    % Expand graph until depth or target is reached (TODO)
    while (length(dfsearch(search_graph, 1)) < search_depth) ...
            && (euclidean_distance(node, target_pose) > 1)
        
        % Advance tree expansion to next node
        parent = parent + 1;
        parent_trim = search_graph.Nodes{parent, 3};
               
        for i = 1:trim_size
            
            if (~isempty(maneuvers{parent_trim, i}))
                
                % Update node table values
                id = id + 1;
                value = 0;
                trim = i;
                x = x + maneuvers{parent_trim, i}.dx;
                y = y + maneuvers{parent_trim, i}.dy;
                yaw = yaw + maneuvers{parent_trim, i}.dyaw;
          
                node = table(id, value, trim, x, y, yaw);
                search_graph = addnode(search_graph, node);
                search_graph = addedge(search_graph, parent, id);
            end
            
        end
        
    end
    
    h = plot(search_graph);
end

