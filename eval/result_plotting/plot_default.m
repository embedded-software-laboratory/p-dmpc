function plot_default(results, optional)

    arguments
        results (1, 1) struct;
        optional.do_export (1, 1) logical = true;
    end

    plot_mpa(results.mpa, results.options, do_export = optional.do_export);
    plot_mpa_over_time(results.mpa, results.options, do_export = optional.do_export);
    plot_mpa_local_reachable_sets(results.mpa, results.options, do_export = optional.do_export);

    plot_computation_time(results, do_export = optional.do_export);

    plot_trajectories(results, do_export = optional.do_export);

    plot_scenario(results.scenario, results.options, do_export = optional.do_export);
end
