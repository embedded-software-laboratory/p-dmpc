% checks whether the vehicles have reached their goal states (only poses)
% @found is boolean
function found = is_goal(curPoses, targetPoses, goalOffsets)

    n_elements = length(goalOffsets);
    
    found = ones(n_elements,1);
    
    for i = 1 : n_elements
       
        if euclidean_distance(curPoses(i),targetPoses(i)) > goalOffsets(i)
        
            found(i) = false;
            
        end
        
    end

end

