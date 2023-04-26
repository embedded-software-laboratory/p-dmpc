function manual_control(vehicle_id, input_device_id, control_mode)
    %   MANUAL_CONTROL      Main function for handling the manual input from multiple devices with/without force feedback
    %                       implements functionality for differenct control modes

    n_manual_vehicle = length(vehicle_id);

    assert(length(input_device_id) == n_manual_vehicle);
    assert(length(control_mode) == n_manual_vehicle);

    manual_controller = cell(n_manual_vehicle, 1);

    % initialize manual controller in their assigned control mode
    for i = 1:n_manual_vehicle

        switch control_mode(i)
            case ControlMode.ManualMode
                manual_controller{i} = ManualMode(vehicle_id(i), input_device_id(i));
            case ControlMode.SemiAutonomousMode
                manual_controller{i} = SemiAutonomousMode(); %TODO %
        end

    end

    %     rate = ros2rate(manual_controller{1}.joy_node, ManualControl.rate_hz);
    % only possible from R2022b
    % main control loop
    while true
        t_start = tic;

        for i = 1:n_manual_vehicle

            try
                input_device_data = manual_controller{i}.decode_input_data();

                [result, force_feedback] = manual_controller{i}.MessageProcessing(input_device_data);

                manual_controller{i}.apply(result, force_feedback);
            catch e
                warning(e.message)
            end

        end

        t_loop = toc(t_start);
        pause(ManualControl.dt_seconds - t_loop);
    end

end
