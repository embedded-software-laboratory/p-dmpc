% checks whether the vehicles have reached their goal states (only poses)
% @found is boolean
function found = is_goal(cur_poses, target_poses, goal_offsets)

    n_elements = length(goal_offsets);
    
    found = ones(n_elements,1);
    
    for i = 1 : n_elements
       
        if euclidean_distance(cur_poses(i),target_poses(i)) > goal_offsets(i)
        
            found(i) = false;
            
        end
        
    end

end

