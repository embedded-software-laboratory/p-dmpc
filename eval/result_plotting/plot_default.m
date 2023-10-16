function plot_default(result, options)

    arguments
        result (1, 1);
        options.do_export (1, 1) logical = false;
    end

    plot_mpa(result.mpa, result.scenario, ...
        do_export = options.do_export);
    plot_mpa_over_time(result.mpa, result.scenario, ...
        do_export = options.do_export);
    plot_mpa_local_reachable_sets(result.mpa, ...
        do_export = options.do_export);

end
