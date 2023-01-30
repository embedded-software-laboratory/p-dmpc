function manual_control_acceleration_experiment(vehicle_id, input_device_id)

    manual_controller = ManualMode(vehicle_id, input_device_id);

    t_exp_start = 0;
    t_exp_brake = 3;
    t_exp_end = t_exp_brake + 2;
    t = 0;
    while t < t_exp_end
        t_loop_start = tic;
        
        try
            input_device_data = manual_controller.decode_input_data();

            if (t_exp_start==0)
                t_exp_start = input_device_data.timestamp;
            end
            t = 1e-9 * double(input_device_data.timestamp - t_exp_start);
    
            input_device_data.steering = 0;
            if (t<t_exp_brake)
                input_device_data.throttle = 1;
                input_device_data.brake = 0;
            else
                input_device_data.throttle = 0;
                input_device_data.brake = 1;
            end
    
            [result, force_feedback] = manual_controller.MessageProcessing(input_device_data);
    
            manual_controller.apply(result, force_feedback);
        catch e
            warning(e.message);
        end
        disp(t)

        t_loop = toc(t_loop_start);
        pause(ManualControl.dt_seconds - t_loop);
    end
end
