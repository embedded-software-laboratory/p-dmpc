function plot_default(result, options)

    arguments
        result (1, 1) struct;
        options.do_export (1, 1) logical = true;
    end

    plot_mpa(result.mpa, result.scenario, do_export = options.do_export);
    plot_mpa_over_time(result.mpa, result.scenario, do_export = options.do_export);
    plot_mpa_local_reachable_sets(result.mpa, result.scenario, do_export = options.do_export);

    plot_computation_time(result, do_export = options.do_export);

    plot_trajectories(result, do_export = options.do_export);

    plot_scenario(result.scenario, do_export = options.do_export);
    step_indices = [1, 5, 9];
    plot_experiment_snapshots(result, step_indices, do_export = options.do_export);
end
