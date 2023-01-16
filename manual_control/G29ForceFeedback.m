classdef G29ForceFeedback

    properties (Access = private)
        g29_pub; % publisher
        force_feedback_message; % initialize message type
        steering_over_time = zeros(1, 3); % for computing steering derivatives
    end

    methods

        function obj = G29ForceFeedback()
            % function to initialize ROS 2 publisher for ros_g29_force_feedback

            g29_node = ros2node("/g29");

            msgList = ros2("msg", "list"); % get all ROS 2 message types

            if sum(cellfun(@(c)strcmp(c, 'ros_g29_force_feedback/ForceFeedback'), msgList)) == 0

                [file_path, ~, ~] = fileparts(mfilename('fullpath'));
                path_custom_msg = [file_path, filesep, '../commun/cust2'];

                % Generate custom messages. Requirements:
                % https://de.mathworks.com/help/ros/gs/ros-system-requirements.html
                % for more details according to your MATLAB version).
                ros2genmsg(path_custom_msg)
            end

            % TODO necessary?
            % setenv LD_LIBRARY_PATH < matlabroot >/ extern / bin / glnxa64: < matlabroot >/ sys / os / glnxa64

            obj.force_feedback_message = ros2message('ros_g29_force_feedback/ForceFeedback');

            obj.g29_pub = ros2publisher(g29_node, '/ff_target', 'ros_g29_force_feedback/ForceFeedback');
        end

        function send_message(obj, data)
            % send desired position and torque to steering wheel
            obj.force_feedback_message.position = single(data.position);
            obj.force_feedback_message.torque = single(data.torque);
            send(obj.g29_pub, obj.force_feedback_message);
        end

        % TODO use or remove
        function [steering_speed, steering_acceleration] = compute_steering_derivatives(obj, steering)
            obj.steering_over_time(3) = steering;
            obj.steering_over_time = circshift(obj.steering_over_time, 1);
            dt = ManualControl.dt_seconds;
            d_speed = diff(obj.steering_over_time);
            steering_speed = mean(d_speed / dt);
            dd_speed = diff(d_speed);
            steering_acceleration = dd_speed / dt ^ 2;
        end

        function result = compute_force_feedback_manual_mode(obj, vehicle_state, steering)

            arguments
                obj (1,1) G29ForceFeedback
                vehicle_state (1, 1) VehicleState
                steering (1, 1) double
            end

            speed_max = 2.22;
            % TODO choose a variant
            % torque = 0.1 ...
            % + 0.4 * abs(steering) * sin(vehicle_state.speed / speed_max * pi / 2) ...
            %     + 0.5 * sin(vehicle_state.speed / speed_max * pi / 2);


            % Separate in torque that acts towards center and torque that acts outwards
            speed_min_torque = 0.1;
            if vehicle_state.speed < speed_min_torque
                [steering_speed, ~] = compute_steering_derivatives(obj, steering);
                if sign(steering_speed) ~= sign(steering)
                    result.position = -sign(steering_speed);
                else
                    result.position = 0;
                end
            end
            % more than linear increase with speed, degressive increase with steering
            torque = 0.2 ...
                + 0.8 * (vehicle_state.speed ./speed_max) .* ...
                (0.3 + 0.7*(sin(abs(steering)) * pi / 2));

            torque = min(1, (10 * steering) ^ 2) * torque; % for smooth torque increase at 0 degree
            result.torque = torque;
        end

        function result = compute_force_feedback_semi_autonomous_mode(vehicle_state)
            result = struct( ...
                'position', vehicle_state.steering_servo, ...
                'torque', 0.22 ...
            );
        end

    end

end
