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
    result.subcontroller_runtime_all_grps = cell(0,1);
    result.subcontroller_runtime_max = zeros(0,1);
    result.parl_groups_infos = cell(0,1);
    
    % create output directory
    directory_name = strrep(strcat(result.scenario.name, '_', result.scenario.controller_name),' ','_');
    output_path = fullfile('results', directory_name);
    result.output_path = output_path;
    if ~isfolder(output_path)
        mkdir(output_path);
    end
end

