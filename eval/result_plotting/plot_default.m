function plot_default(experiment_results, optional)

    arguments
        experiment_results (1, :) ExperimentResult;
        optional.do_export (1, 1) logical = true;
    end

    experiment_result = merge_experiment_results(experiment_results);

    plot_mpa(experiment_result.mpa, experiment_result.options, do_export = optional.do_export);
    plot_mpa_over_time(experiment_result.mpa, experiment_result.options, do_export = optional.do_export);
    plot_mpa_local_reachable_sets(experiment_result.mpa, experiment_result.options, do_export = optional.do_export);

    %some function still use the array of ExperimentResults for plotting
    plot_computation_time(experiment_results, do_export = optional.do_export);

    plot_trajectories(experiment_result, do_export = optional.do_export);

    plot_scenario(experiment_result.scenario, experiment_result.options, do_export = optional.do_export);
end
