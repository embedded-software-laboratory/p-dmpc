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
        else
            laneID = 0;
        end
    end
end