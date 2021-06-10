% checks whether the vehicles have reached their goal positions
% @result is boolean
function result = is_goal(cur_node, scenario)
    idx = Tree.nodeCols();
    d = vecnorm([ cur_node(:,idx.x) - [scenario.vehicles(:).x_goal]' , ...
                  cur_node(:,idx.y) - [scenario.vehicles(:).y_goal]' ]...
                , 2, 2 ...
    );
    result = (d < scenario.r_goal);
end

