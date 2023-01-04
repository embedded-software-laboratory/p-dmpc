function manual_control(vehicle_id, input_device_id, control_mode)
    
    n_manual_vehicle = lenght(vehicle_id);

    assert(length(input_device_id) == n_manual_vehicle);
    assert(length(control_mode) == n_manual_vehicle);

    manual_controller = cell(n_manual_vehicle, 1);

    for i = 1:length(vehicle_id)
        if control_mode(i) == 1
            manual_controller(i) = ManualMode(vehicle_id, input_device_id);
        else
            if control_mode(i) == 2
                manual_controller(i) = SemiAutonomousMode(vehicle_id, input_device_id);
            end
        end
    end

    
end