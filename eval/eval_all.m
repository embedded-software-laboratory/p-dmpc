function eval_all(optional)

    arguments
        optional.shut_down (1, 1) logical = false
    end

    eval_prioritization(computation_mode = ComputationMode.parallel_physically);
    eval_grouping(computation_mode = ComputationMode.parallel_physically);

    if optional.shut_down
        script_path = fullfile(pwd, 'nuc_control', 'shutdown_nuc.sh');
        command = ['bash ', script_path];
        [~, ~] = system(command);
    end

end
