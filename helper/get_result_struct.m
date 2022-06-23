function result = get_result_struct(scenario)
% GET_RESULT_STRUCT     Initialize result struct returned by simulation.

    result = struct;
    
    result.scenario = scenario;
    result.controller_outputs = cell(0,1);
    result.iteration_structs = cell(0,1);
    result.vehicle_path_fullres = cell(scenario.nVeh,0);
    result.trajectory_predictions = cell(scenario.nVeh,0);
    result.controller_runtime = zeros(0,1);
    result.step_time = zeros(0,1);
    result.n_expanded = zeros(0,1);
    result.subcontroller_runtime_all_grps = cell(0,1); % subcontroller runtime of all parallel groups
    result.subcontroller_run_time_total = zeros(0,1); % subcontroller total runtime
    result.belonging_vector = cell(0,1); % belonging vector indicates to which groups one vehicle belongs
    
    % create output directory and get the full path where the results will
    % be saved
    results_full_path = FileNameConstructor.get_results_full_path(...
        scenario.name,...
        scenario.controller_name,...
        scenario.trim_set,...
        scenario.Hp,...
        scenario.nVeh,...
        scenario.T_end,...
        scenario.options.isParl,...
        scenario.max_num_CLs,...
        scenario.strategy_consider_veh_without_ROW,...
        scenario.strategy_enter_intersecting_area);
    
    result.output_path = results_full_path;

end

