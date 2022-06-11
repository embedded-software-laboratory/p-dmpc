classdef GuidedMode

    properties(Access=private)
        axes
        buttons
        steering
        throttle
        brake
        leftPaddle
        rightPaddle
    end

    properties(Access=public)
        scenario
        updatedPath
    end

    methods
        function modeHandler = GuidedMode(scenario, x_measured, mVehid, vehid, cooldown_after_lane_change, speedProfileMPAsInitialized, inputData, steeringWheel)
            modeHandler.axes = inputData.axes;
            modeHandler.buttons = inputData.buttons;

            if (steeringWheel)
                modeHandler.steering = modeHandler.axes(1);
                modeHandler.throttle = modeHandler.axes(3);
                modeHandler.brake = modeHandler.axes(4);
                modeHandler.leftPaddle = modeHandler.buttons(6);
                modeHandler.rightPaddle = modeHandler.buttons(5);
            else
                modeHandler.steering = modeHandler.axes(1);
                modeHandler.throttle = modeHandler.axes(3);
                modeHandler.brake = modeHandler.axes(6);
                modeHandler.leftPaddle = modeHandler.buttons(5);
                modeHandler.rightPaddle = modeHandler.buttons(6);
            end

            % ID's for the speed profiles in choose_trims.m
            low_speed_profile_trim_set = 2;
            high_speed_profile_trim_set = 6;
            mediumSpeedProfileMPA = scenario.mpa;
            scenario.speed_profile_mpas(2) = mediumSpeedProfileMPA;

            % precompute MPA's to save computation time
            if ~speedProfileMPAsInitialized
                lowSpeedProfileMPA = MotionPrimitiveAutomaton(...
                    scenario.model...
                    , low_speed_profile_trim_set...
                    , scenario.offset...
                    , scenario.dt...
                    , scenario.nVeh...
                    , scenario.Hp...
                    , scenario.tick_per_step...
                    , true...
                    , scenario.is_allow_non_convex...
                    , scenario.options...
                );
                scenario.speed_profile_mpas(1) = lowSpeedProfileMPA;

                highSpeedProfileMPA = MotionPrimitiveAutomaton(...
                    scenario.model...
                    , high_speed_profile_trim_set...
                    , scenario.offset...
                    , scenario.dt...
                    , scenario.nVeh...
                    , scenario.Hp...
                    , scenario.tick_per_step...
                    , true...
                    , scenario.is_allow_non_convex...
                    , scenario.options...
                );
                scenario.speed_profile_mpas(3) = highSpeedProfileMPA;
            end

            laneID = 0;
            idx = indices();

            for k = 1:length(vehid)
                if vehid(k) == mVehid
                    vehicle_iteration_index = k;
                    break
                end
            end

            % if steering over threshold, then perform lane change
            if cooldown_after_lane_change > 5
                if modeHandler.steering > 0.3
                    disp("entered steering left");
                    % find out current lane of manual vehicle
                    index = match_pose_to_lane(scenario, x_measured(vehicle_iteration_index, idx.x), x_measured(vehicle_iteration_index, idx.y));
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
                    index = match_pose_to_lane(scenario, x_measured(vehicle_iteration_index, idx.x), x_measured(vehicle_iteration_index, idx.y));
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
                [updated_ref_path, scenario] = generate_manual_path(scenario, mVehid, 20, laneID, false); 
                
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

            % select higher speed profile if not already max speed
            if modeHandler.leftPaddle == 1
                if scenario.vehicles(vehicle_iteration_index).paddle_counter < 3
                    disp("entered up shift");
                    scenario.vehicles(vehicle_iteration_index).paddle_counter = scenario.vehicles(vehicle_iteration_index).paddle_counter + 1;

                    scenario.vehicles(vehicle_iteration_index).vehicle_mpa = scenario.speed_profile_mpas(scenario.vehicles(vehicle_iteration_index).paddle_counter);

                    if steeringWheel
                        scenario.manual_mpa_initialized = true;
                    else
                        scenario.second_manual_mpa_initialized = true;
                    end
                end
            end

            % select lower speed profile if not already min speed
            if modeHandler.rightPaddle == 1
                if scenario.vehicles(vehicle_iteration_index).paddle_counter > 1
                    disp("entered down shift");
                    scenario.vehicles(vehicle_iteration_index).paddle_counter = scenario.vehicles(vehicle_iteration_index).paddle_counter - 1;
                    
                    scenario.vehicles(vehicle_iteration_index).vehicle_mpa = scenario.speed_profile_mpas(scenario.vehicles(vehicle_iteration_index).paddle_counter);

                    if steeringWheel
                        scenario.manual_mpa_initialized = true;
                    else
                        scenario.second_manual_mpa_initialized = true;
                    end  
                end
            end

            modeHandler.scenario = scenario;
        end
    end

end