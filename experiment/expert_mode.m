function expert_mode(manual_vehicle_id, force_feedback_enabled)

    % Initialize data readers/writers...
    % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
    common_cpm_functions_path = fullfile( ...
        getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
    );
    assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
    addpath(common_cpm_functions_path);

    matlabDomainId = 1;
    [matlabParticipant,~,~,~,~,~,~,writer_vehicleCommandDirect] = init_script(matlabDomainId); % #ok<ASGLU>

    % Middleware period for valid_after stamp
    dt_period_nanos = uint64(4*1e7);

    % create subscriber
    wheelNode = ros2node("/wheel");
    wheelSub = ros2subscriber(wheelNode,"/j0","sensor_msgs/Joy");
    pause(0.2);

    if force_feedback_enabled
        g29_handler = G29ForceFeedback();
        g29_last_position = 0.0;
    end

    t_start = tic;
    %r = rosrate(0.01);

    while(true)

        %waitfor(r);
        wheel_message = wheelSub.LatestMessage;

        wheelData = struct;
        wheelData.axes = wheel_message.axes;
        wheelData.buttons = wheel_message.buttons;
        wheelData.steering =  wheelData.axes(1);
        wheelData.throttle = wheelData.axes(3);
        wheelData.brake = wheelData.axes(4);
        wheelData.leftPaddle = wheelData.buttons(6);
        wheelData.rightPaddle = wheelData.buttons(5);
    
        vehicle_command_direct = VehicleCommandDirect;
        vehicle_command_direct.vehicle_id = uint8(manual_vehicle_id);

        throttle = 0;
        if wheelData.throttle >= 0
            throttle = wheelData.throttle;
        elseif wheelData.brake >= 0
            throttle = (-1) * wheelData.brake;
        end

        wheelData.steering = 1.2 * wheelData.steering;

        if throttle > 0.5
            throttle = 0.5;
        elseif throttle < -0.5
            throttle = -0.5;
        end

        disp(sprintf("throttle: %f, steering: %f", throttle, wheelData.steering));
        vehicle_command_direct.motor_throttle = double(throttle);
        vehicle_command_direct.steering_servo = double(wheelData.steering);

        timestamp_sec = uint64(wheel_message.header.stamp.sec);
        timestamp_nanosec = uint64(wheel_message.header.stamp.nanosec);
        timestamp = uint64(timestamp_sec*1e9 + timestamp_nanosec);

        t = toc(t_start);

        if t > 0.01
            t_start = tic;

            vehicle_command_direct.header.create_stamp.nanoseconds = ...
                uint64(timestamp);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(timestamp+dt_period_nanos);
            writer_vehicleCommandDirect.write(vehicle_command_direct);

        end
    
        if force_feedback_enabled
            g29_last_position = g29_handler.g29_send_message(0.0, 0.5, g29_last_position);
        end
    end
    
end
