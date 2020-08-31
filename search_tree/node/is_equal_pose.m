% gets two poses and a offset 
% creates a square with midpoint of pose 2 and lengths of 2*offset
% checks if pose 1 is in this square
% @boolean is true if pose 1 is in this square
function isequal = is_equal_pose(x1,y1,yaw1,x2,y2,yaw2,offset)
    
    isequal = (abs(x1-x2) <= offset) && (abs(y1-y2) <= offset) && (abs(yaw1-yaw2) <= offset);

end

