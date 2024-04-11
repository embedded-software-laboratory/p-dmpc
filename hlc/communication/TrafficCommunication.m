classdef TrafficCommunication < InterHlcCommunication
    % communication class for Traffic message

    properties
        pose_indices (1, 1) struct % indices of x, y, heading, speed
    end

    methods

        function obj = TrafficCommunication( ...
                vehicle_index, ...
                ros2_node, ...
                topic_name, ...
                message_type ...
            )

            arguments
                vehicle_index (1, 1) double
                ros2_node (1, 1) ros2node
                topic_name (1, :) char
                message_type (1, :) char
            end

            % create communication class to connect to ROS2 network
            % call superclass constructor
            obj = obj@InterHlcCommunication( ...
                vehicle_index, ...
                ros2_node, ...
                topic_name, ...
                message_type ...
            );
            % struct that hold the indices for x, y, heading, speed
            % it is stored as member variable to create it only once
            obj.pose_indices = indices();
        end

        function send_message( ...
                obj, ...
                time_step, ...
                current_pose, ...
                current_trim_index, ...
                current_lanelet, ...
                predicted_lanelets, ...
                reference_trajectory_points, ...
                occupied_areas, ...
                reachable_sets ...
            )

            arguments
                obj (1, 1) TrafficCommunication
                time_step (1, 1) double
                current_pose (1, 4) double
                current_trim_index (1, 1) double
                current_lanelet (1, 1) double
                predicted_lanelets (1, :) double
                reference_trajectory_points (:, 2) double
                occupied_areas (1, 1) struct
                reachable_sets (1, :) cell
            end

            % vehicle send message to its topic
            obj.message_to_be_sent.time_step = int32(time_step);
            obj.message_to_be_sent.vehicle_index = int32(obj.vehicle_index);
            obj.message_to_be_sent.current_pose.x = current_pose(obj.pose_indices.x);
            obj.message_to_be_sent.current_pose.y = current_pose(obj.pose_indices.y);
            obj.message_to_be_sent.current_pose.heading = current_pose(obj.pose_indices.heading);
            obj.message_to_be_sent.current_pose.speed = current_pose(obj.pose_indices.speed);
            obj.message_to_be_sent.current_trim_index = int32(current_trim_index);
            obj.message_to_be_sent.current_lanelet = int32(current_lanelet);
            obj.message_to_be_sent.predicted_lanelets = int32(predicted_lanelets);

            for i = 1:size(reference_trajectory_points, 1)
                obj.message_to_be_sent.reference_trajectory_points(i).x = reference_trajectory_points(i, 1);
                obj.message_to_be_sent.reference_trajectory_points(i).y = reference_trajectory_points(i, 2);
                obj.message_to_be_sent.reference_trajectory_points(i).z = 0;
            end

            % occupied areas of current time step.
            % normal offset at index 1, without offset at index 2
            obj.message_to_be_sent.occupied_areas(1).x = occupied_areas.normal_offset(1, :);
            obj.message_to_be_sent.occupied_areas(1).y = occupied_areas.normal_offset(2, :);
            obj.message_to_be_sent.occupied_areas(2).x = occupied_areas.without_offset(1, :);
            obj.message_to_be_sent.occupied_areas(2).y = occupied_areas.without_offset(2, :);

            % comment out if vehicles send their reachable sets to others
            for i = 1:length(reachable_sets)
                obj.message_to_be_sent.reachable_sets(i).x = reachable_sets{i}(1, :);
                obj.message_to_be_sent.reachable_sets(i).y = reachable_sets{i}(2, :);
            end

            send(obj.ros2_publisher, obj.message_to_be_sent);
        end

    end

end
