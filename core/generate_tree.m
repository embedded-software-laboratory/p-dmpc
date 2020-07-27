% Generates search tree of all feasible trajectories
% init_pose refers to the struct representing the initial pose of our vehicle
% target_pose refers to the struct representing the target pose of our vehicle
% trim refers to the index of the initial trim
% maneuvers refers to the matrix of maneuvers of a motion graph
% search_depth specifies the depth of the created search tree
function search_graph = generate_tree(init_pose, target_pose, trim_index, maneuvers, search_depth)
    
    % Initialize node table values
    id =  1;
    value = euclidean_distance(init_pose, target_pose);
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
    
    % Array storing ids of nodes that may be expanded
    leaf_nodes = [node.id];
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while (length(dfsearch(search_graph, 1)) < search_depth) ...
            && (euclidean_distance(node, target_pose) > 1 ...
            && ~isempty(leaf_nodes))
        
        % Advance tree expansion to next node by choosing closest leaf to
        % target 
        if (length(leaf_nodes) > 1)
            
            min_value = search_graph.Nodes{leaf_nodes(1), 2};
            parent = leaf_nodes(1);
            
            for i = 2:length(leaf_nodes)
                
                if min_value > search_graph.Nodes{leaf_nodes(i), 2}
                    min_value = search_graph.Nodes{leaf_nodes(i), 2};
                    parent = leaf_nodes(i);
                end
                
            end
            
            % Delete chosen entry from list of expandable nodes
            leaf_nodes(leaf_nodes == parent) = [];
            
        else
            parent = leaf_nodes(1);
            leaf_nodes(1) = [];
        end
        
        parent_trim = search_graph.Nodes{parent, 3};
               
        for i = 1:trim_size
            
            if (~isempty(maneuvers{parent_trim, i}))
              
                % Update node table values
                id = id + 1;
                trim = i;
                x = x + maneuvers{parent_trim, i}.dx;
                y = y + maneuvers{parent_trim, i}.dy;
                yaw = yaw + maneuvers{parent_trim, i}.dyaw;
                
                % Update pose for value calculation
                cur_pose.x = x;
                cur_pose.y = y;
                cur_pose.yaw = yaw;
                value = euclidean_distance(cur_pose, target_pose);
          
                % Add node to existing new graph and connect parent to it
                node = table(id, value, trim, x, y, yaw);
                search_graph = addnode(search_graph, node);
                search_graph = addedge(search_graph, parent, id);
                
                % Update new leaves to be expanded 
                leaf_nodes = [leaf_nodes node.id];
                
            end
            
        end
        
        % Reset parent 
        parent = NaN;
     
    end
    
    h = plot(search_graph);
end

