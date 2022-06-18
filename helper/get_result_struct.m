function result = get_result_struct(scenario, options)
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
    
    % create output directory


    directory_name = strrep(strcat(result.scenario.name, '_', result.scenario.controller_name),' ','_');

    [file_path,~,~] = fileparts(mfilename('fullpath')); % get the path of the current file
    idcs = strfind(file_path,filesep); % find all positions of '/'
    one_folder_up = file_path(1:idcs(end)-1); % one folder up

    folder_target = fullfile(one_folder_up,'results',directory_name);
    if ~isfolder(folder_target)
        % create target folder if not exist
        mkdir(folder_target)
    end
    if options.isParl
        file_name = ['trims',num2str(scenario.trim_set),'_Hp',num2str(scenario.Hp),'_nVeh',num2str(scenario.nVeh),'_T',num2str(scenario.T_end),'_maxCLs',num2str(scenario.max_num_CLs),'.mat'];
    else
        file_name = ['trims',num2str(scenario.trim_set),'_Hp',num2str(scenario.Hp),'_nVeh',num2str(scenario.nVeh),'_T',num2str(scenario.T_end),'.mat'];
    end
    output_path = fullfile(folder_target,file_name);
    
    result.output_path = output_path;
    result.output_folder = folder_target;

end

