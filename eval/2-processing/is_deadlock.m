%returns vector v of size n_steps
%v_i is 1 if any vehicle is in deadlock at iteration step i
%v_i is 0 otherwise
function deadlock = is_deadlock(experiment_result)

    arguments
        experiment_result (1, 1) ExperimentResult
    end

    mpa = MotionPrimitiveAutomaton(experiment_result.options);

    if isempty(mpa)
        error('error! ExperimentResult must contain mpa, make sure to turn off reduce_result option in config!')
    end

    % calculate deadlock
    % if a vehicle stops for more than a defined time, assume deadlock
    deadlock = zeros(experiment_result.n_steps, 1);
    vehs_stop_duration = zeros(experiment_result.options.amount, 1);

    for k = 1:experiment_result.n_steps
        % vehicles that stop at the current time step
        is_vehicle_stopped = ismember(experiment_result.iteration_data(k).trim_indices, mpa.trims_stop);
        % increase couter of vehicles that stop
        vehs_stop_duration(is_vehicle_stopped) = ...
            vehs_stop_duration(is_vehicle_stopped) + 1;
        % reset vehicles that do not stop anymore
        vehs_stop_duration(~is_vehicle_stopped) = 0;

        % check for deadlock
        threshold_stop_steps = 3 * experiment_result.options.Hp;
        is_vehicle_deadlocked = (vehs_stop_duration > threshold_stop_steps);

        deadlock(k) = any(is_vehicle_deadlocked);
    end

end
