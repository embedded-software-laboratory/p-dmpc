function plot_default(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult;
        optional.do_export (1, 1) logical = true;
    end

    plot_mpa(experiment_result.mpa, experiment_result.options, do_export = optional.do_export);
    plot_mpa_over_time(experiment_result.mpa, experiment_result.options, do_export = optional.do_export);
    plot_mpa_local_reachable_sets(experiment_result.mpa, experiment_result.options, do_export = optional.do_export);

    plot_trajectories(experiment_result, do_export = optional.do_export);

    plot_scenario(experiment_result.options, do_export = optional.do_export);
end
