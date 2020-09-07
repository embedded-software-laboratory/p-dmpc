function [search_tree, parents] = combine_trees(init_poses, target_poses, trim_indices, combined_graph, search_depth, is_collisionF, graph_searchF)
%COMBINE_TREES Combines multiple runs
[search_tree, parents] = generate_tree(init_poses, target_poses, trim_indices, combined_graph, search_depth, is_collisionF, graph_searchF);
n_veh = length(combined_graph.motionGraphList);
offset = ones(1, n_veh);

poses = [];
for i = 1:n_veh
    pose.x = search_tree.Node{end}.xs(i);
    pose.y = search_tree.Node{end}.ys(i);
    pose.yaw = search_tree.Node{end}.yaws(i);
    poses = [poses, pose];
end

is_goals = is_goal(poses, target_poses, offset);

while(sum(is_goals) ~= n_veh)
    
    for i = 1:n_veh
        poses(i).x = search_tree.Node{end}.xs(i);
        poses(i).y = search_tree.Node{end}.ys(i);
        poses(i).yaw = search_tree.Node{end}.yaws(i);
    end
    trims = search_tree.Node{end}.trims;
    
    target_node = search_tree.Parent(end);
    last_node = search_tree.nnodes();
    
    search_tree = search_tree.removenode(last_node);
    
    % Generate new branch and add to existing
    [search_branch, parents_branch] = generate_tree(poses, target_poses, trims, combined_graph, search_depth, is_collisionF, graph_searchF);
    parents_branch = parents_branch + last_node;
    parents = [parents parents_branch];
        
    search_tree = search_tree.graft(target_node, search_branch);
    
    is_goals = is_goal(poses, target_poses, offset);
end

end

