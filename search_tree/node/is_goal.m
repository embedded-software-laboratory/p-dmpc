% checks whether the vehicles have reached their goal states (only poses)
% @found is boolean
function found = is_goal(cur_node, target_poses)   
    distance = euclidean_distance(cur_node,target_poses);
    found = (distance < offset);
end

