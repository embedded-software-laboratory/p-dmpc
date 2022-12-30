function [options, vehicle_ids] = eval_guided_mode(collisionAvoidanceMode)
%% evaluate Guided Mode for specified collision avoidance mode

    options = struct;
    options.collisionAvoidanceMode = collisionAvoidanceMode;
    options.is_sim_lab = false;
    options.is_mixed_traffic = true;
    options.scenario_name = 'Commonroad';

    if collisionAvoidanceMode == 1
        vehicle_ids = [1,2,3,5,6,7,9,10,11,13,14,15];
    else
        vehicle_ids = [1,3,5,7,9,11,13,15];
    end

    options.manualVehicle_id = '1';
    options.firstManualVehicleMode = '1';
    options.force_feedback_enabled = true;
    options.manualVehicle_id2 = '9';
    options.secondManualVehicleMode = '1';
    options.mixed_traffic_config.consider_rss = false;
    
    options.Hp = 6;
    options.trim_set = 4;
    options.T_end = 30;
    options.dt = 0.2;
    options.max_num_CLs = 4;
    options.strategy_consider_veh_without_ROW = '2';
    options.strategy_enter_lanelet_crossing_area = '1';


    % workaround to generate repeatable random numbers
    % use 3 to allow for lane changes of autonomous vehicles into the intersection, and 20220705 instead
    %rng(20220705,'twister');
    rng(3,'twister');

    test = randi(1);
    options.seed = rng;
    test = randi(1);

    pathToScript = fullfile(pwd,'/eval','/eval_mixed_traffic','eval_script.sh');
    cmdStr = ['gnome-terminal --' ' ' pathToScript];
    system(cmdStr);

end
