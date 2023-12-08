function [n_steps, t] = compute_deadlock_free_runtime(experiment_result)
    % COMPUTE_DEADLOCK_FREE_RUNTIME computes the deadlock-free runtime of a experiment_result
    arguments
        experiment_result (1, 1) ExperimentResult
    end

    n_steps = find(is_deadlock(experiment_result), 1, 'first');

    if isempty(n_steps)
        t = experiment_result.t_total;
        n_steps = experiment_result.n_steps;
    else
        t = n_steps * experiment_result.options.dt_seconds;
    end

end
