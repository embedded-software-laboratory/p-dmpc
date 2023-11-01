function plot_default(results, options)

    arguments
        results (1, 1) struct;
        options.do_export (1, 1) logical = true;
    end

    plot_mpa(results.mpa, results.scenario, do_export = options.do_export);
    plot_mpa_over_time(results.mpa, results.scenario, do_export = options.do_export);
    plot_mpa_local_reachable_sets(results.mpa, results.scenario, do_export = options.do_export);

    plot_computation_time(results, do_export = options.do_export);

    plot_trajectories(results, do_export = options.do_export);

    plot_scenario(results.scenario, do_export = options.do_export);
end
