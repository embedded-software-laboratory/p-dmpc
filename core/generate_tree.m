% Generates search tree of all feasible trajectories
% init_pose refers to the struct representing the initial pose of our vehicle
% target_pose refers to the struct representing the target pose of our vehicle
% trim refers to the index of the initial trim
% maneuvers refers to the matrix of maneuvers of a motion graph
% search_depth specifies the depth of the created search tree
function search_tree = generate_tree(init_poses, target_poses, trim_indices, combined_graph, search_depth, is_collisionF, graph_searchF, fig, axis_size)
    
    n_veh = length(combined_graph.motionGraphList);
    
    trim_length = [];
    
    for i = 1 : n_veh
    
        tmp_length = length(combined_graph.motionGraphList(i).trims);
        
        trim_length = [trim_length, tmp_length];
    
    end

    % Initialize node table values
    id = 1;
    
    gvalues = zeros(1, n_veh);
    
    goal = zeros(n_veh, 1);
    
    % high but should be unnecessary
    hvalues = Inf(1,n_veh);
    
    trims = trim_indices;
    
    xs = [init_poses(1:n_veh).x];
    ys = [init_poses(1:n_veh).y];
    yaws = [init_poses(1:n_veh).yaw];
    
    cur_poses = init_poses;
    
    parent = 0;
    
    cur_poses = [];
        
    for i = 1 : n_veh

        cur_pose.x = xs(i);
        cur_pose.y = ys(i);

        cur_poses = [cur_poses, cur_pose];

    end
    
    offset = ones(1, n_veh);
    
    isgoals = is_goal(cur_poses, target_poses, offset);
    
    % Create digraph with root node
    node1 = node(id, 0, trims, xs, ys, yaws, gvalues, hvalues);
    search_tree = tree(node1);
    
    % Array storing ids of nodes that may be expanded
    leaf_nodes = [node1.id];
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    tf1 = (search_tree.depth() < search_depth);
    tf2 = (sum(isgoals) == n_veh);
    tf3 = isempty(leaf_nodes);
    
    
    % --- begin of visualization ---
    assignin('base','vis',fig);
    % --- end of visualizaiton ---
    
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while   tf1 ...
            && ~tf2 ...
            && ~tf3
        
        % get next node for expansion
        parent = graph_searchF(search_tree, leaf_nodes);
        
        % Delete chosen entry from list of expandable nodes
        leaf_nodes(leaf_nodes == parent) = [];
        
        [leaf_nodes, search_tree, id, isgoals] = expand_tree(leaf_nodes, search_tree, parent, combined_graph, trim_length, target_poses, visited_nodes, id, isgoals, is_collisionF);
        
        visited_nodes = [visited_nodes, parent];
        
        % --- begin of visualization ---
        parent_node = search_tree.get(parent);
        
        col = 'mcg';

        figure(fig);
        for i = 1 : n_veh
           plot(parent_node.xs(i), parent_node.ys(i), 'o','Color', col(i));
           hold on
           axis(axis_size);
        end

        pause(0.2);
        
        if sum(isgoals) == n_veh
            end_node = search_tree.Node{end};
            figure(fig)
            for i = 1 : n_veh
                plot(end_node.xs(i), end_node.ys(i), 'o','Color', col(i));
                hold on
                axis(axis_size);
            end
        end
        % --- end of visualization ---
        
        
        parent = NaN;

        tf1 = (search_tree.depth() < search_depth);
        tf2 = (sum(isgoals) == n_veh);
        tf3 = isempty(leaf_nodes);

    end 

end

