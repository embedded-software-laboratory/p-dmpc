function experiment_result = main(options, optional)
    % MAIN  main function for graph-based receding horizon control

    arguments
        options (1, 1) Config = config_gui();
        % vehicle IDs of vehicles in the CPM Lab
        optional.vehicle_ids (:, 1) double = 1:options.amount;
    end

    if isMATLABReleaseOlderThan('R2023a')
        warning("Code is developed in MATLAB R2023a, prepare for backward incompatibilities.")
    end

    options.save_to_file();

    % create scenario and write it to disk (with default name)
    scenario = Scenario.create(options);
    save(Config().scenario_file, 'scenario');

    % create dry run scenario and write it to disk
    HlcFactory.create_dry_run_scenario(options);

    % inform where experiment takes place
    if options.environment == Environment.Simulation
        disp('Running in MATLAB simulation...')
    else
        disp('Running in Lab...')
    end

    if options.is_prioritized
        % In prioritized computation, vehicles communicate via ROS 2.
        % Generate the ros2 msgs types.
        generate_ros2_msgs();
    end

    plotter = PlotterOnline(options, scenario);

    if options.computation_mode == ComputationMode.sequential ...
            || ~options.is_prioritized ...
            || options.amount == 1

        hlc = HlcFactory.get_hlc(options, 1:options.amount, do_dry_run = options.should_do_dry_run);
        experiment_result = hlc.run();
        plotter.plotting_loop();

    elseif options.computation_mode == ComputationMode.parallel_threads
        % simulate distribution locally using the Parallel Computing Toolbox
        get_parallel_pool(options.amount);

        future(1:options.amount) = parallel.FevalFuture;

        for i_vehicle = 1:options.amount
            future(i_vehicle) = parfeval( ...
                @main_distributed, ...
                1, ...
                i_vehicle, ...
                options = options ...
            );
        end

        plotter.plotting_loop();

        experiment_result = fetchOutputs(future, UniformOutput = false);

    elseif options.computation_mode == ComputationMode.parallel_physically
        push_files_to_nuc();
        % On changes to ROS messages or MPA library, remove files with `remove_cache_nuc()`
        deploy_nuc(vehicle_ids = optional.vehicle_ids);
        fprintf('Running experiment...\n');
        plotter.plotting_loop();
        fprintf('Running experiment done.\n')
        experiment_result = collect_results_nuc( ...
            options = options, ...
            vehicle_ids = optional.vehicle_ids ...
        );
    end

    if numel(experiment_result) > 1
        experiment_result = merge_experiment_results([experiment_result{:}]);
    end

    experiment_result = experiment_result.add_meta_information();
    experiment_result.save_merged();
end
