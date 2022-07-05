function [options, vehicle_ids] = eval_guided_mode(collisionAvoidanceMode)
%% evaluate mixed traffic for first collision avoidance mode

    options = struct;
    options.collisionAvoidanceMode = collisionAvoidanceMode;
    options.is_sim_lab = false;
    options.is_mixed_traffic = true;
    options.scenario = 'Commonroad';

    vehicle_ids = [2,4,6,10,12,14];
    options.manualVehicle_id = '2';
    options.firstManualVehicleMode = '1';
    options.force_feedback_enabled = false;
    options.manualVehicle_id2 = '10';
    options.secondManualVehicleMode = '1';
    options.consider_RSS = false;
    
    options.Hp = 6;
    options.trim_set = 4;
    options.T_end = 30;
    options.dt = 0.2;
    options.max_num_CLs = 4;
    options.strategy_consider_veh_without_ROW = '2';
    options.strategy_enter_intersecting_area = '1';

    % workaround to generate repeatable random numbers
    rng(20220705,'twister');
    test = randi(1);
    options.seed = rng;
    test = randi(1);

    pathToScript = fullfile(pwd,'/eval','/eval_mixed_traffic','eval_script.sh');
    cmdStr = ['gnome-terminal --' ' ' pathToScript];
    system(cmdStr);


    %cd /usr/local/MATLAB/R2022a/bin
    %./matlab -sd /home/david/dev/software/high_level_controller/graph_based_planning -r eval_script
    %cmdStr = ['gnome-terminal --' ' ' '/urs/local/MATLAB/R2022a/bin ./matlab' ' ' '-sd' ' ' '/home/david/dev/software/high_level_controller/graph_based_planning' ' ' '-r' ' ' 'eval_script'];
    %system(cmdStr);

end
