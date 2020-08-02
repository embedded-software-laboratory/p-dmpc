% checks whether the vehicles have reached their goal states (only poses)
% @found is boolean
function found = is_goal(curPoses, targetPoses, goalOffsets)

    found = true;

    n_elements = length(goalOffsets);
    
    for i = 1 : n_elements
       
        if euclidean_distance(curPoses(i),targetPoses(i)) > goalOffsets(i)
        
            found = false;
            return;
            
        end
        
    end

end

