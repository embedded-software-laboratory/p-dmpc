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
    
    % set plot limits based on min and max values of reference trajectory
    % of all involved vehicles + padding in all direction
    pad = 0.5;
    for veh = scenario.vehicles
        max_x = max(veh.referenceTrajectory(:,1)+pad);
        min_x = min(veh.referenceTrajectory(:,1)-pad);
        max_y = max(veh.referenceTrajectory(:,2)+pad);
        min_y = min(veh.referenceTrajectory(:,2)-pad);
    end
    result.plot_limits = [min_x,max_x;min_y,max_y];
end

