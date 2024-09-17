function push_files_to_nuc()
    fprintf('Updating NUC files...'); tic;

    % update all nuc files
    script_path = fullfile(pwd, 'nuc_control', 'push_files_to_nuc.sh');

    command = ['bash ', script_path];

    [~, ~] = system(command);

    fprintf(' done (%.2f s).\n', toc)
end
