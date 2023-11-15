function result = main(options)
    % MAIN  main function for graph-based receding horizon control

    arguments
        options (1, 1) Config = start_options();
    end

    if verLessThan('matlab', '9.12')
        warning("Code is developed in MATLAB 2022a, prepare for backward incompatibilities.")
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

    is_prioritized_parallel_in_lab = (options.is_prioritized && (options.environment == Environment.CpmLab || options.environment == Environment.SimulationDistributed) && options.compute_in_parallel);

    if is_prioritized_parallel_in_lab
        disp('Scenario was written to disk. Select main_distributed(vehicle_id) in LCC next.')

        delete_ros2_msgs();
    else
        % get plant from options
        plant = PlantFactory.get_experiment_interface(options.environment);

        % set active vehicle IDs and possibly initialize communication
        plant.setup(options, scenario, options.path_ids);

        if options.is_prioritized
            % In priority-based computation, vehicles communicate via ROS 2.
            % Generate the ros2 msgs types.
            generate_ros2_msgs();
        end

        if options.is_prioritized == true && options.compute_in_parallel
            %% simulate distribution locally using the Parallel Computing Toolbox
            get_parallel_pool(options.amount);

            do_plot = options.options_plot_online.is_active;
            can_handle_parallel_plot = isa(plant, 'SimLab');

            if do_plot

                if can_handle_parallel_plot
                    visualization_data_queue = plant.get_visualization_data_queue;
                    % create central plotter - used by all workers via data queue
                    plotter = PlotterOnline(options, scenario, plant.indices_in_vehicle_list);
                    afterEach(visualization_data_queue, @plotter.data_queue_callback);
                else
                    warning('The currently selected environment cannot handle plotting of a parallel execution!');
                end

            end

            spmd (options.amount)
                % setup plant again, only control one vehicle
                plant.setup(options, scenario, options.path_ids, options.path_ids(labindex));

                hlc_factory = HLCFactory();
                % have the plant only control its own vehicle by calling setup a second time
                hlc = hlc_factory.get_hlc(options, scenario, plant, plant.controlled_vehicle_ids, options.is_dry_run);
                result = hlc.run();
            end

            if do_plot
                plotter.close_figure();
            end

            result = {result{:}};
        else

            hlc_factory = HLCFactory();
            hlc = hlc_factory.get_hlc(options, scenario, plant, plant.controlled_vehicle_ids, options.is_dry_run);
            result = hlc.run();
        end

    end

end
