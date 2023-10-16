function plot_default(hlc, options)

    arguments
        hlc (1, 1) HighLevelController;
        options.do_export (1, 1) logical = false;
    end

    plot_mpa(hlc.mpa, hlc.scenario, ...
        do_export = options.do_export);
    plot_mpa_over_time(hlc.mpa, hlc.scenario, ...
        do_export = options.do_export);
    plot_mpa_local_reachable_sets(hlc.mpa, ...
        do_export = options.do_export);

end
