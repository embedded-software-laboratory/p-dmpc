function result = get_result_struct(scenario, controlled_vehicle_ids)
    % GET_RESULT_STRUCT     Initialize result struct returned by simulation.

    result = struct;

    % initialize fields that are iteratively updated
    result.nSteps = 0;
    result.t_total = 0;

    result.options = scenario.options;
    result.scenario = scenario;
    result.vehicle_ids = controlled_vehicle_ids;
    result.controller_outputs = cell(0, 1);
    result.iteration_structs = cell(0, 1);
    result.vehicle_path_fullres = cell(scenario.options.amount, 0);
    result.trajectory_predictions = cell(scenario.options.amount, 0);
    result.belonging_vector = zeros(scenario.options.amount, 0); % belonging vector indicates to which group one vehicle belongs to
    result.priority_list = zeros(scenario.options.amount, 0);
    result.computation_levels = zeros(1, 0);
    result.vehs_fallback = cell(0, 1); % which vehicles should use their fallback trajectories

end
