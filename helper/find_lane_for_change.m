function laneID = find_lane_for_change(index_successor, turnLeft)
    load('commonroad_data.mat');
    commonroad = commonroad_data;

    if turnLeft
        successor_adjacentLeft = commonroad_data.lanelet(index_successor).adjacentLeft;

        if isfield(successor_adjacentLeft,'refAttribute') && strcmp(successor_adjacentLeft.drivingDirAttribute,'same')
            laneID = successor_adjacentLeft.refAttribute;  
        else
            laneID = 0;
        end
    else
        successor_adjacentRight = commonroad_data.lanelet(index_successor).adjacentRight;
        
        if isfield(successor_adjacentRight,'refAttribute') && strcmp(successor_adjacentRight.drivingDirAttribute,'same')
            laneID = successor_adjacentRight.refAttribute;
        % hardcode values to get from outer circle to crossing
        elseif index_successor == 5
            laneID = 23;
        elseif index_successor == 7
            laneID = 9;
        elseif index_successor == 27
            laneID = 41;
        elseif index_successor == 29
            laneID = 48;
        elseif index_successor == 53
            laneID = 67;
        elseif index_successor == 55
            laneID = 74;
        elseif index_successor == 83
            laneID = 101;
        elseif index_successor == 85
            laneID = 87;
        else
            laneID = 0;
        end
    end
end