classdef GuidedMode

    properties(Access=private)
        axes
        buttons
        steering
        throttle
        brake
    end

    properties(Access=public)
        scenario
        updatedPath
    end

    methods
        function modeHandler = GuidedMode(scenario, x_measured, mVehid, vehid, cooldown_after_lane_change, wheelData)
            modeHandler.axes = wheelData.axes;
            modeHandler.buttons = wheelData.buttons;

            modeHandler.steering = modeHandler.axes(1);
            modeHandler.throttle = modeHandler.axes(3);
            modeHandler.brake = modeHandler.axes(4);

            laneID = 0;
            idx = indices();

            for k = 1:length(vehid)
                if vehid(k) == mVehid
                    vehicle_iteration_index = k;
                    break
                end
            end

            % if steering over threshold, then perform lane change
            if cooldown_after_lane_change > 9
                if modeHandler.steering > 0.3
                    disp("entered steering left");
                    % find out current lane of manual vehicle
                    index = match_pose_to_lane(x_measured(vehid(vehicle_iteration_index), idx.x), x_measured(vehid(vehicle_iteration_index), idx.y));
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
                    end

                elseif modeHandler.steering < -0.3
                    disp("entered steering right");
                    % find out current lane of manual vehicle
                    index = match_pose_to_lane(x_measured(vehid(vehicle_iteration_index), idx.x), x_measured(vehid(vehicle_iteration_index), idx.y));
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
                    end
                end
            end

            if (laneID ~= 0)
                disp(sprintf("laneID: %d", laneID));
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

                modeHandler.updatedPath = true;
            else
                modeHandler.updatedPath = false;
            end 

            modeHandler.scenario = scenario;
        end
    end

end