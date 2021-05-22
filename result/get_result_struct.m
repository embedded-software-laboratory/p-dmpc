function result = get_result_struct(scenario)
    result = struct;
    
    result.scenario = scenario;
    result.controller_outputs = cell(0,1);
    result.iteration_structs = cell(0,1);
    result.vehicle_path_fullres = cell(scenario.nVeh,0);
    result.trajectory_predictions = cell(scenario.nVeh,0);
    result.controller_runtime = zeros(0,1);
    result.step_time = zeros(0,1);
    result.n_expanded = zeros(scenario.nVeh,1);
    
    % create output directory
    directory_name = strrep(strcat(result.scenario.name, '_', result.scenario.controller_name),' ','_');
    output_path = fullfile('output', directory_name);
    result.output_path = output_path;
    if ~isfolder(output_path)
        mkdir(output_path);
    end
    
    % set plot limits based on min and max on the start positions
    % of all involved vehicles + padding in all direction
    % not optimal but better than before
    pad = 0.5;
    veh = scenario.vehicles;
    max_x = max([veh(:).x_start]+pad);
    min_x = min([veh(:).x_start]-pad);
    max_y = max([veh(:).y_start]+pad);
    min_y = min([veh(:).y_start]-pad);
    result.plot_limits = [min_x,max_x;min_y,max_y];
end

