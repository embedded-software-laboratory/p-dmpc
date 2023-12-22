function push_files_to_remote()
    fprintf('Updating NUC files...');

    % update all nuc files
    script_path = fullfile(pwd, 'nuc_simulation', 'push_files_to_remote.sh');

    command = ['bash ', script_path];

    [~, ~] = system(command);

    fprintf(' done.\n')
end
