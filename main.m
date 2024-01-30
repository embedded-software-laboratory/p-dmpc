function experiment_result = main(options)
    % MAIN  main function for graph-based receding horizon control

    arguments
        options (1, 1) Config = config_gui();
    end

    if isMATLABReleaseOlderThan('R2023a')
        warning("Code is developed in MATLAB R2023a, prepare for backward incompatibilities.")
    end

    % create scenario and write it to disk (with default name)
    scenario = create_scenario(options);
    save(Config().scenario_file, 'scenario');

    % create dry run scenario and write it to disk
    HlcFactory.create_dry_run_scenario(options);

    % inform where experiment takes place
    if options.environment == Environment.Simulation
        disp('Running in MATLAB simulation...')
    else
        disp('Running in Lab...')
    end

    if options.computation_mode == ComputationMode.parallel_physically
        disp('Scenario was written to disk. If run with LCC, call `delete_ros2_msgs()` before.')
        return
    end

    if options.is_prioritized
        % In prioritized computation, vehicles communicate via ROS 2.
        % Generate the ros2 msgs types.
        generate_ros2_msgs();
    end

    if options.options_plot_online.is_active
        plotter = PlotterOnline(options, scenario);
    end

    if ~options.is_prioritized ...
            || options.computation_mode == ComputationMode.sequential ...
            || options.amount == 1
        experiment_result = run_hlc(options, 1:options.amount);

        if options.options_plot_online.is_active
            plotter.plotting_loop();
        end

    else
        % simulate distribution locally using the Parallel Computing Toolbox
        get_parallel_pool(options.amount);

        future(1:options.amount) = parallel.FevalFuture;

        for i_vehicle = 1:options.amount
            future(i_vehicle) = parfeval(@run_hlc, 1, options, i_vehicle);
        end

        if options.options_plot_online.is_active
            plotter.plotting_loop();
        end

        experiment_result = fetchOutputs(future, UniformOutput = false);

    end

    if numel(experiment_result) > 1
        experiment_result = merge_experiment_results([experiment_result{:}]);
    end

    experiment_result = experiment_result.add_meta_information();
    experiment_result.save_merged();
end

function experiment_result = run_hlc(options, i_vehicle)
    hlc = HlcFactory.get_hlc(options, i_vehicle);
    experiment_result = hlc.run();
end
