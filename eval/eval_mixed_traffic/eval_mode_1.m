function [options, vehicle_ids] = eval_mode_1()
%% evaluate mixed traffic for first collision avoidance mode

    options = struct;
    options.collisionAvoidanceMode = 1;
    options.is_sim_lab = false;
    options.is_mixed_traffic = true;
    options.scenario = 'Commonroad';

    vehicle_ids = [2,4,6,8,10,12,14,16];
    options.manualVehicle_id = '1';
    options.firstManualVehicleMode = '1';
    options.force_feedback_enabled = false;
    options.manualVehicle_id2 = '9';
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
    rng(1,'twister');
    test = randi(1);
    options.seed = rng;
    test = randi(1);

    pathToScript = fullfile(pwd,'/eval','/eval_mixed_traffic','eval_script.sh');
    seed = num2str(options.seed.Seed);
    cmdStr = ['gnome-terminal --' ' ' pathToScript ' ' seed];
    system(cmdStr);


    %results_full_path = FileNameConstructor.get_results_full_path(scenario_name,controller_name,trim_sest,...
                    %Hp,nVeh,T_end,isParl,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_intersecting_area);
    %evaluation = EvaluationCommon(results_full_path);
    %evaluation = evaluation.get_path_tracking_errors;
    %evaluation = evaluation.get_total_runtime_per_step;

end
