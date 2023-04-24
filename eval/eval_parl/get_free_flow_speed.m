function free_flow_speed = get_free_flow_speed(dt)
%GET_FREE_FLOW_SPEED Free flow speed is the speed that vehicles could travel if they are not hindered by others
% Inputs
%   dt: sample time [s]
% Outputs
%   free_flow_speed: free flow speed [m/s]

    % prepare simulation options
    options = Config();
    options.environment = Environment.Simulation;
    options.customResultName = '';
    options.scenario_name = 'Commonroad';
    options.trim_set = 9;
    options.Hp = 5;
    options.priority = 'STAC_priority';
    options.T_end = 20;
    options.isPB = true;
    options.isAllowInheritROW = false;
    options.strategy_consider_veh_without_ROW = '3';
    options.strategy_enter_lanelet_crossing_area = '4';
    options.isSaveResult = true;
    options.options_plot_online = OptionsPlotOnline();
    options.is_eval = false;
    options.amount = 20; % could also be other numbers
    options.max_num_CLs = options.amount;
    options.is_free_flow = true;
    options.dt = dt;
    options.isParl = false; % avoid using distributed computation
    options.veh_ids = 1:options.amount;
    
    results_full_path = FileNameConstructor.get_results_full_path(options,options.amount);
    if isfile(results_full_path)
        disp('File already exists.')
    else
        % run simulation
        [~,~] = main(options);
    end

    % evaluate
    evaluation = EvaluationParl(results_full_path);
    
    free_flow_speed = evaluation.average_speed;
    disp(['Free-flow speed for sample time ' num2str(dt) ' s: ' num2str(free_flow_speed) ' m/s.'])

end