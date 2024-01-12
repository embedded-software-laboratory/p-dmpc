function remove_cash_remote()
    fprintf('Removing cashed NUC files...');

    % remove all nuc files
    script_path = fullfile(pwd, 'nuc_simulation', 'remove_cash_remote.sh');

    command = ['bash ', script_path];

    [~, ~] = system(command);

    fprintf(' done.\n')
end
