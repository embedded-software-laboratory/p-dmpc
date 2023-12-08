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
    if options.environment == Environment.Simulation || options.environment == Environment.SimulationDistributed
        disp('Running in MATLAB simulation...')
    else
        disp('Running in Lab...')
    end

    is_prioritized_parallel_in_lab = ( ...
        options.is_prioritized && ...
        options.compute_in_parallel && ...
        (options.environment == Environment.CpmLab || options.environment == Environment.SimulationDistributed) ...
    );

    if is_prioritized_parallel_in_lab
        delete_ros2_msgs();
        disp('Scenario was written to disk. Select main_distributed(vehicle_id) in LCC next.')
        return
    end

    if options.is_prioritized
        % In priority-based computation, vehicles communicate via ROS 2.
        % Generate the ros2 msgs types.
        generate_ros2_msgs();
    end

    if ~options.is_prioritized || ~options.compute_in_parallel
        % get plant from options
        plant = Plant.get_plant(options.environment);

        % set active vehicle IDs and possibly initialize communication
        plant.setup(options, options.path_ids);

        hlc_factory = HLCFactory();
        hlc = hlc_factory.get_hlc(options, plant, plant.controlled_vehicle_ids, options.is_dry_run);
        experiment_result = hlc.run();

    else

        % simulate distribution locally using the Parallel Computing Toolbox
        get_parallel_pool(options.amount);

        do_plot = options.options_plot_online.is_active;
        can_handle_parallel_plot = options.environment == Environment.Simulation;

        if do_plot

            if can_handle_parallel_plot
                visualization_data_queue = parallel.pool.DataQueue;
                % create central plotter - used by all workers via data queue
                plotter = PlotterOnline(options, scenario, 1:options.amount);
                afterEach(visualization_data_queue, @plotter.data_queue_callback);
            else
                % case for UnifiedLabApi (and for CpmLab if combination with Parallel Computing Toolbox would be possible)
                warning('The currently selected environment cannot handle plotting of a parallel execution!');
            end

        end

        spmd (options.amount)
            % get plant from options
            plant = Plant.get_plant(options.environment);

            if do_plot && can_handle_parallel_plot
                % set visualization_data_queue for plotting
                plant.set_visualization_data_queue(visualization_data_queue);
            end

            % setup plant again, only control one vehicle
            plant.setup(options, options.path_ids, options.path_ids(spmdIndex));

            hlc_factory = HLCFactory();
            % have the plant only control its own vehicle by calling setup a second time
            hlc = hlc_factory.get_hlc(options, plant, plant.controlled_vehicle_ids, options.is_dry_run);
            experiment_result = hlc.run();
        end

        if do_plot && can_handle_parallel_plot
            plotter.close_figure();
        end

        experiment_result = experiment_result(:);
    end

end
