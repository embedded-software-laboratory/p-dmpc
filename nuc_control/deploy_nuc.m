function deploy_nuc(optional)

    arguments
        optional.vehicle_ids (:, 1) double;
    end

    if isempty(optional.vehicle_ids)
        optional.vehicle_ids = 1:Config.load_from_file('Config.json').amount;
    end

    vehicle_ids_arg = sprintf(' %d', optional.vehicle_ids);

    fprintf('Starting remote HLCs...'); tic;

    script_path = fullfile(pwd, 'nuc_control', 'deploy_nuc.sh');
    log_path = fullfile(pwd, 'nuc_control', 'deploy_nuc.log');
    command = ['bash ', script_path, vehicle_ids_arg, ' &> ', log_path];
    [~, ~] = system(command);

    fprintf(' done (%.2f s).\n', toc)
end
