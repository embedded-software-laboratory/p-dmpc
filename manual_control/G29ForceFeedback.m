classdef G29ForceFeedback < handle
    % G29FORCEFEEDBACK      Class handling everything about force feedback for the Logitech G29 Gaming Wheel

    properties (Access = private)
        g29_pub; % publisher
        force_feedback_message; % initialize message type
    end

    methods

        function obj = G29ForceFeedback()
            % function to initialize ROS 2 publisher for ros_g29_force_feedback

            g29_node = ros2node("/g29");

            msgList = ros2("msg", "list"); % get all ROS 2 message types

            if sum(cellfun(@(c)strcmp(c, 'ros_g29_force_feedback/ForceFeedback'), msgList)) == 0

                [file_path, ~, ~] = fileparts(mfilename('fullpath'));
                path_custom_msg = file_path;

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

        function result = compute_force_feedback_manual_mode(obj, speed, steering)

            arguments
                obj (1, 1) G29ForceFeedback
                speed (1, 1) double
                steering (1, 1) double
            end

            speed_max = 2.22;

            % more than linear increase with speed, degressive increase with steering
            torque = 0.15 ...
                + 0.7 * (speed ./ speed_max) .* ...
                (0.3 + 0.7 * (sin(abs(steering)) * pi / 2));
            result.position = 0;

            % for smooth torque increase at 0 degree
            torque = min(1, (10 * steering)^2) * torque;
            result.torque = torque;
        end

        function result = compute_force_feedback_semi_autonomous_mode(vehicle_state)
            result = struct( ...
                'position', vehicle_state.steering_servo, ...
                'torque', 0.22 ...
            );
        end

        function plot_force_feedback(obj)
            % plot force feedback for different steering angles and speeds
            steering = linspace(-1, 1, 100);
            speed = linspace(0, 2.22, 100);

            torque = zeros(length(steering), length(speed));

            for i_steering = 1:length(steering)

                for i_speed = 1:length(speed)
                    data = obj.compute_force_feedback_manual_mode(speed(i_speed), steering(i_steering));
                    torque(i_steering, i_speed) = data.torque;
                end

            end

            f = figure(100);
            [speed_mesh, steering_mesh] = meshgrid(speed, steering);
            surf(speed_mesh, steering_mesh, torque ...
                , 'EdgeColor', 'none' ...
            );
            xlabel("Speed [m/s]");
            ylabel("Steering [-]");
            zlabel("Torque [-]");
            [rwth_colormap, ~] = discrete_colormap();
            rwth_colormap = flip(rwth_colormap, 1);
            colormap(rwth_colormap);
            set_figure_properties(f, ExportFigConfig.paper("paperheight", 6));
            export_fig(f, "results/force_feedback.pdf");
            export_fig(f, "results/force_feedback.emf");
        end

    end

end
