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
    result.belonging_vector = cell(0,1); % belonging vector indicates to which groups one vehicle belongs
    result.priority = zeros(scenario.nVeh,0);
    result.computation_levels = zeros(1,0);
    result.vehs_fallback = cell(0,1); % which vehicles should use their fallback trajectories
    
    % create output directory and get the full path where the results will
    % be saved
    results_full_path = FileNameConstructor.get_results_full_path(scenario.options);
    
    result.output_path = results_full_path;

end

