function result = get_result_struct(scenario, depth)
    result = struct;
    
    result.scenario = scenario;
    result.controller_outputs = cell(depth,1);
    result.iteration_structs = cell(depth,1);
    result.vehicle_path_fullres = cell(scenario.nVeh,depth);
    result.trajectory_predictions = cell(scenario.nVeh,depth);
    result.controller_runtime = zeros(depth,1);
    result.step_time = zeros(depth,1);
end

