classdef OptionsMain
    % OPTIONSMAIN Options for main.m

    properties
        manualVehicle_id        % one of the following: {'1','2',...,'nVeh-1','nVeh'}, defines which vehicle is the first manual vehicle
        firstManualVehicleMode  % one of the following: {'1','2'}, '1' for guided-mode while '2' for expert-mode
        manualVehicle_id2       % one of the following: {'1','2',...,'nVeh-1','nVeh'}, defines which vehicle is the second manual vehicle
        secondManualVehicleMode % one of the following: {'1','2'}, '1' for guided-mode while '2' for expert-mode
        collisionAvoidanceMode  % one of the following: {1,2,3}, defines collision avoid mode, 1 for priority-based, 2 for reachability analysis guided-mode, while 3 for reachability analysis expert-mode
        is_sim_lab              % true/false, is simulation or lab experiment
        is_mixed_traffic        % true/false, is mixed traffic
        force_feedback_enabled  % true/false
        consider_RSS    % true/false, is consider Responsibility-Sensitive Safety
        isPB            % true/false, is prioritize vehicles
        angles          % 1-by-nVeh scalar vector
        amount          % integer, number of vehicles
        visu            % 1-by-2 vector, online plotting is enabled if the first entry if true; node visualization is enabled if the second entry is true
        isParl          % true/false, is use parallel computation
        scenario        % one of the follows: {'Circle_scenario','Commonroad'}
        priority        % one of the following: {'topo_priority','right_of_way_priority','constant_priority','random_priority','FCA_priority'}, defines which priority assignmen strategy is used
        dt              % scalar, sample time
        Hp              % scalar, prediction horizon
        trim_set        % scalar, ID of trim primitive
        T_end           % scalar, simulation duration
        max_num_CLs     % integer, maximum allowerd number of computation levels
        strategy_consider_veh_without_ROW       % one of the following: {'1','2','3','4','5'}, strategy of letting higher-priority vehicles consider their coupled vehicles with lower priorities
        strategy_enter_lanelet_crossing_area    % one of the following: {'1','2','3','4'}, strategy of forbidding vehicles with lower priorities entering their lanelet crossing area
        isSaveResult            % true/false, is save result
        customResultName        % string or char, custom file name to save result
        isAllowInheritROW       % true/false, is allow vehicles to inherit the right-of-way from their front vehicles
        is_eval                 % true/false, 
        visualize_reachable_set % true/false, 
    end

    methods
        function obj = OptionsMain()
        end

        function obj = assign_data(obj,options)
            % Assign data to options (see an example at the bottom)
            obj.manualVehicle_id = options.manualVehicle_id;
            obj.firstManualVehicleMode = options.firstManualVehicleMode;
            obj.manualVehicle_id2 = options.manualVehicle_id2;
            obj.secondManualVehicleMode = options.secondManualVehicleMode;
            obj.collisionAvoidanceMode = options.collisionAvoidanceMode;
            obj.is_sim_lab = options.is_sim_lab;
            obj.is_mixed_traffic = options.is_mixed_traffic;
            obj.force_feedback_enabled = options.force_feedback_enabled;
            obj.consider_RSS = options.consider_RSS;
            obj.isPB = options.isPB;
            obj.angles = options.angles;
            obj.amount = options.amount;
            obj.visu = options.visu;
            obj.isParl = options.isParl;
            obj.scenario = options.scenario;
            obj.priority = options.priority;
            obj.dt = options.dt;
            obj.Hp = options.Hp;
            obj.trim_set = options.trim_set;
            obj.T_end = options.T_end;
            obj.max_num_CLs = options.max_num_CLs;
            obj.strategy_consider_veh_without_ROW = options.strategy_consider_veh_without_ROW;
            obj.strategy_enter_lanelet_crossing_area = options.strategy_enter_lanelet_crossing_area;
            obj.isSaveResult = options.isSaveResult;
            obj.customResultName = options.customResultName;
            obj.isAllowInheritROW = options.isAllowInheritROW;
            obj.is_eval = options.is_eval;
            obj.visualize_reachable_set = options.visualize_reachable_set;
        end
    end
end

%% Example
% options = struct;
% options.manualVehicle_id = '1';
% options.firstManualVehicleMode = '1';
% options.manualVehicle_id2 = '2';
% options.secondManualVehicleMode = '1';
% options.collisionAvoidanceMode = 1;
% options.is_sim_lab = true;
% options.is_mixed_traffic = false;
% options.force_feedback_enabled = true;
% options.consider_RSS = false;
% options.isPB = true;
% options.angles = [];
% options.amount = 3;
% options.visu = [true,false];
% options.isParl = false;
% options.scenario = 'Commonroad';
% options.priority = 'right_of_way_priority';
% options.dt = 0.2;
% options.Hp = 5;
% options.trim_set = 9;
% options.T_end = 30;
% options.max_num_CLs = 4;
% options.strategy_consider_veh_without_ROW = '3';
% options.strategy_enter_lanelet_crossing_area = '4';
% options.isSaveResult = false;
% options.customResultName = '';
% options.isAllowInheritROW = true;
% options.is_eval = false;
% options.visualize_reachable_set = false;

% optionsMain = OptionsMain().assign_data(options)
