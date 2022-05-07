classdef GuidedMode

    properties(Access=private)
        scenario
        axes
        buttons
        steering
        throttle
        brake
    end

    methods
        function modeHandler = GuidedMode(scenario, x_measured, mVehid, vehid, wheelData)
            modeHandler.axes = wheelData.axes;
            modeHandler.buttons = wheelData.buttons;

            modeHandler.steering = modeHandler.axes(1);
            modeHandler.throttle = modeHandler.axes(3);
            modeHandler.brake = modeHandler.axes(4);

            % if steering over threshold, then perform lane change
            if modeHandler.steering < -0.5
                % find out current lane of manual vehicle
                index = match_pose_to_lane(x_measured(mVehid, idx.x), x_measured(mVehid, idx.y));
                index_successor = 0;

                for iVeh = 1:scenario.nVeh
                    if (mVehid == vehid(iVeh))
                        lanelets = scenario.vehicles(iVeh).lanelets_index;

                        % find out successor in reference path
                        for j = 1:(length(lanelets) - 1)
                            if lanelets(j) == index
                                index_successor = lanelets(j+1);
                            end
                        end

                        if (index_successor == 0)
                            disp("error when finding successor of current lane");
                        end

                        iteration_index = iVeh;
                        break
                    end
                end

                if (index_successor ~= 0)
                    laneID = find_lane_for_change(index_successor, true);
                else
                    laneID = 0;
                end

            elseif modeHandler > 0.5
                % find out current lane of manual vehicle
                index = match_pose_to_lane(x_measured(mVehid, idx.x), x_measured(mVehid, idx.y));
                index_successor = 0;

                for iVeh = 1:scenario.nVeh
                    if (mVehid == vehid(iVeh))
                        lanelets = scenario.vehicles(iVeh).lanelets_index;

                        % find out successor in reference path
                        for j = 1:(length(lanelets) - 1)
                                if lanelets(j) == index
                                    index_successor = lanelets(j+1);
                                end
                        end

                        if (index_successor == 0)
                            disp("error when finding successor of current lane");
                        end

                        iteration_index = iVeh;
                        break
                    end
                end

                if (index_successor ~= 0)
                    laneID = find_lane_for_change(index_successor, false);
                else
                    laneID = 0;
                end
            end

            if (laneID ~= 0)
                % generate manual path with lane as start index
                % maybe current lane has to be start value of lane -> has to be evaluated
                updated_ref_path = generate_manual_path(scenario, mVehid, 20, laneID); 
                
                updatedRefPath = updated_ref_path.path;
                scenario.vehicles(iteration_index).x_start = updatedRefPath(1,1);
                scenario.vehicles(iteration_index).y_start = updatedRefPath(1,2);
                scenario.vehicles(iteration_index).x_goal = updatedRefPath(2:end,1);
                scenario.vehicles(iteration_index).y_goal = updatedRefPath(2:end,2);
                
                scenario.vehicles(iteration_index).referenceTrajectory = [scenario.vehicles(iteration_index).x_start scenario.vehicles(iteration_index).y_start
                                        scenario.vehicles(iteration_index).x_goal  scenario.vehicles(iteration_index).y_goal];
                scenario.vehicles(iteration_index).lanelets_index = updated_ref_path.lanelets_index;
                scenario.vehicles(iteration_index).points_index = updated_ref_path.points_index;

                yaw = calculate_yaw(updatedRefPath);
                scenario.vehicles(iteration_index).yaw_start = yaw(1);
                scenario.vehicles(iteration_index).yaw_goal = yaw(2:end); 
            end  

            modeHandler.scenario = scenario;
        end

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

    end

end