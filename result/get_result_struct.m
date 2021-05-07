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
    
    % set plot limits based on min and max values of reference trajectory
    % of all involved vehicles + padding in all direction
    max_x = 0;
    min_x = 0;
    max_y = 0;
    min_y = 0;
    pad = 2;
    for veh = scenario.vehicles
        max_x = max([veh.referenceTrajectory(:,1)+pad;max_x]);
        min_x = min([veh.referenceTrajectory(:,1)-pad;min_x]);
        max_y = max([veh.referenceTrajectory(:,2)+pad;max_y]);
        min_y = min([veh.referenceTrajectory(:,2)-pad;min_y]);
    end
    result.plot_limits = [min_x,max_x;min_y,max_y];
end

