function result = get_result_struct(hlc)
    % GET_RESULT_STRUCT     Initialize result struct returned by simulation.

    scenario = hlc.scenario;
    result = struct;

    result.scenario = scenario;
    result.controller_outputs = cell(0, 1);
    result.iteration_structs = cell(0, 1);
    result.vehicle_path_fullres = cell(scenario.options.amount, 0);
    result.trajectory_predictions = cell(scenario.options.amount, 0);
    result.controller_runtime = zeros(0, 1);
    result.step_time = zeros(0, 1);
    result.belonging_vector = zeros(scenario.options.amount, 0); % belonging vector indicates to which group one vehicle belongs to
    result.priority_list = zeros(scenario.options.amount, 0);
    result.computation_levels = zeros(1, 0);
    result.vehs_fallback = cell(0, 1); % which vehicles should use their fallback trajectories
    result.vehicle_ids = hlc.plant.controlled_vehicle_ids;
    result.t_total = 0;

end
