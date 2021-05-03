function result = get_result_struct(scenario, depth)
    result = struct;
    
    result.scenario = scenario;
    result.controller_outputs = cell(depth,1);
    result.iteration_structs = cell(depth,1);
    result.vehicle_path_fullres = cell(scenario.nVeh,depth);
    result.trajectory_predictions = cell(scenario.nVeh,depth);
    result.controller_runtime = zeros(depth,1);
    result.step_time = zeros(depth,1);
    % create output directory
    directory_name = strrep([result.scenario.name, '_', result.scenario.controller_name],' ','_');
    output_path = strjoin(['output\', directory_name ,'\'],'');
    result.output_path = output_path;
    switch scenario.nVeh
        case 1
            result.plot_limits = [-22,22;-7,7];
        case 2
            result.plot_limits = [-22,22;-7,7];
        otherwise
            result.plot_limits = [-22,22;-22,22];
    end
end

