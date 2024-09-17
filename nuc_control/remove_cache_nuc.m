function remove_cache_nuc()
    fprintf('Removing cashed NUC files...');

    % remove all nuc files
    script_path = fullfile(pwd, 'nuc_control', 'remove_cache_nuc.sh');

    command = ['bash ', script_path];

    [~, ~] = system(command);

    fprintf(' done.\n')
end
