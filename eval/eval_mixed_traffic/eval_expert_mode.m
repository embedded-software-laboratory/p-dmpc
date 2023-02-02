function [options, vehicle_ids] = eval_expert_mode(consider_RSS)
%% evaluate Expert Mode 
    
        options = struct;
        options.collisionAvoidanceMode = 3;
        options.is_sim_lab = false;
        options.is_mixed_traffic = true;
        options.scenario_name = 'Commonroad';
    
        vehicle_ids = [1,3];
        options.manualVehicle_id = '1';
        options.firstManualVehicleMode = '2';
        options.force_feedback_enabled = false;
        options.manualVehicle_id2 = 'No second MV';
        options.secondManualVehicleMode = '1';
        options.mixed_traffic_config.consider_rss = consider_RSS;
        
        options.Hp = 6;
        options.trim_set = 4;
        options.T_end = 30;
        options.dt = 0.2;
        options.max_num_CLs = 4;
        options.strategy_consider_veh_without_ROW = '2';
        options.strategy_enter_lanelet_crossing_area = '1';
    
        % workaround to generate repeatable random numbers
        rng(20220705,'twister');
        test = randi(1);
        options.seed = rng;
        test = randi(1);
    end
    