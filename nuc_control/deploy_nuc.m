function deploy_nuc(optional)

    arguments
        optional.options = Config.load_from_file('Config.json');
        optional.vehicle_ids (:, 1) double = 1:options.amount;
    end

    vehicle_ids_arg = sprintf(' %d', optional.vehicle_ids);

    fprintf('Starting remote HLCs...');

    script_path = fullfile(pwd, 'nuc_control', 'deploy_nuc.sh');
    log_path = fullfile(pwd, 'nuc_control', 'deploy_nuc.log');
    command = ['bash ', script_path, vehicle_ids_arg, ' &> ', log_path];
    [~, ~] = system(command);

    fprintf(' done.\n')
end
