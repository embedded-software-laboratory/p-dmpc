% checks whether the vehicles have reached their goal positions
% @result is boolean
function result = is_goal(cur_node, scenario)
    d = vecnorm([ cur_node(:,NodeInfo.x) - [scenario.vehicles(:).x_goal]' , ...
                  cur_node(:,NodeInfo.y) - [scenario.vehicles(:).y_goal]' ]...
                , 2, 2 ...
    );
    result = (d < scenario.r_goal);
end

