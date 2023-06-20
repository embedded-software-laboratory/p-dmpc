function [result, scenario] = main(varargin)
    % MAIN  main function for graph-based receeding horizon control

    if verLessThan('matlab', '9.12')
        warning("Code is developed in MATLAB 2022a, prepare for backward incompatibilities.")
    end

    % check if Config object is given as input
    options = read_object_from_input(varargin, 'Config');
    % check if Scenario object is given as input
    scenario = read_object_from_input(varargin, 'Scenario');

    % If scenario/options are not given, determine from UI
    if isempty(scenario)

        if isempty(options)

            try
                options = start_options();
            catch ME
                warning(ME.message);
                return
            end

        end

        % if no (or insufficient) path ids were specified, use default values
        if isempty(options.path_ids) || (length(options.path_ids) ~= options.amount)
            warning('amount of reference path ids does not conform to amount of vehicles, resorting to default paths');
            options.path_ids = 1:options.amount;
        end

        plant = PlantFactory.get_experiment_interface(options.environment);
        % create scenario
        random_seed = RandStream('mt19937ar');
        scenario = create_scenario(options, random_seed, plant);
    else
        plant = PlantFactory.get_experiment_interface(scenario.options.environment);
    end

    % write scenario to disk if distributed (for lab or local debugging with main_distributed())
    if scenario.options.is_prioritized == true
        save('scenario.mat', 'scenario');
    end

    is_prioritized_parallel_in_lab = (scenario.options.is_prioritized && scenario.options.environment == Environment.CpmLab && scenario.options.compute_in_parallel);

    if is_prioritized_parallel_in_lab
        disp('Scenario was written to disk. Select main_distributed(vehicle_id) in LCC next.')

        if exist("commun/cust1/matlab_msg_gen", 'dir')

            try
                rmdir("commun/cust1/matlab_msg_gen", 's');
            catch
                warning("Unable to delete commun/cust1/matlab_msg_gen. Please delete manually");
            end

        end

        if exist("commun/cust2/matlab_msg_gen", "dir")

            try
                rmdir("commun/cust2/matlab_msg_gen", 's');
            catch
                warning("Unable to delete commun/cust2/matlab_msg_gen. Please delete manually");
            end

        end

    else
        % set active vehicle IDs and possibly initialize communication
        plant.setup(scenario);

        hlc_factory = HLCFactory();
        hlc_factory.set_scenario(scenario);
        dry_run = (scenario.options.environment == Environment.CpmLab); % TODO: dry run also for unified lab api?

        if scenario.options.use_cpp
            optimizer(Function.CheckMexFunction);
        end

        if scenario.options.is_prioritized == true && scenario.options.compute_in_parallel
            %% simulate distribution locally using the Parallel Computing Toolbox
            get_parallel_pool(scenario.options.amount);

            do_plot = scenario.options.options_plot_online.is_active;
            can_handle_parallel_plot = isa(plant, 'SimLab');

            if do_plot

                if can_handle_parallel_plot
                    visualization_data_queue = plant.set_visualization_data_queue;
                    % create central plotter - used by all workers via data queue
                    plotter = PlotterOnline(hlc_factory.scenario);
                    afterEach(visualization_data_queue, @plotter.data_queue_callback);
                else
                    warning('The currently selected environment cannot handle plotting of a parallel execution!');
                end

            end

            spmd (scenario.options.amount)
                hlc = hlc_factory.get_hlc(scenario.options.veh_ids(labindex), dry_run, plant);
                [result, scenario] = hlc.run();
            end

            if do_plot
                plotter.close_figure();
            end

            result = {result{:}};
            scenario = {scenario{:}};
        else
            hlc = hlc_factory.get_hlc(scenario.options.veh_ids, dry_run, plant);
            [result, scenario] = hlc.run();
        end

    end

end
