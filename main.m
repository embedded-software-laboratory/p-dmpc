function experiment_result = main(options)
    % MAIN  main function for graph-based receding horizon control

    arguments
        options (1, 1) Config = config_gui();
    end

    if isMATLABReleaseOlderThan('R2023a')
        warning("Code is developed in MATLAB R2023a, prepare for backward incompatibilities.")
    end

    % create scenario
    scenario = create_scenario(options);

    % write built scenario to disk
    save('scenario.mat', 'scenario');

    % inform where experiment takes place
    if options.environment == Environment.Simulation
        disp('Running in MATLAB simulation...')
    else
        disp('Running in Lab...')
    end

    if options.computation_mode == ComputationMode.parallel_physically
        delete_ros2_msgs();
        disp('Scenario was written to disk. Select main_distributed(vehicle_id) in LCC next.')
        return
    end

    if options.is_prioritized
        % In priority-based computation, vehicles communicate via ROS 2.
        % Generate the ros2 msgs types.
        generate_ros2_msgs();
    end

    % TODO: sync, but not plot when plotting is not desired
    plotter = PlotterOnline(options, scenario);
    on_cleanup_function = onCleanup(@plotter.close_figure);

    if ~options.is_prioritized || options.computation_mode == ComputationMode.sequential
        run_hlc(options, 1:options.amount);
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

        experiment_result = fetchOutputs(future, 'UniformOutput', false);

    end

end

function experiment_result = run_hlc(options, i_vehicle)
    hlc_factory = HLCFactory();
    % have the plant only control its own vehicle by calling setup a second time
    do_dry_run = false;
    hlc = hlc_factory.get_hlc(options, i_vehicle, do_dry_run);
    experiment_result = hlc.run();
end
