function manual_control(vehicle_id, input_device_id, control_mode)
    
    n_manual_vehicle = length(vehicle_id);

    assert(length(input_device_id) == n_manual_vehicle);
    assert(length(control_mode) == n_manual_vehicle);

    manual_controller = cell(n_manual_vehicle, 1);

    for i = 1:n_manual_vehicle
        switch control_mode(i)
            case ControlMode.ManualMode
                manual_controller{i} = ManualMode(vehicle_id(i), input_device_id(i));
            case ControlMode.SemiAutonomousMode
                manual_controller{i} = SemiAutonomousMode();%TODO%
        end
    end
    
     while true
        for i = 1:n_manual_vehicle
            input_device_data = manual_controller{i}.decode_input_data();
            [result, force_feedback] = manual_controller{i}.MessageProcessing(input_device_data);
            manual_controller{i}.apply(result, force_feedback);
        end
     
    end

    
end