function [result, scenario] = main(varargin)
% MAIN  main function for graph-based receeding horizon control

if verLessThan('matlab','9.12')
    warning("Code is developed in MATLAB 2022a, prepare for backward incompatibilities.")
end


% check if Config object is given as input
options = read_object_from_input(varargin,'Config');
% If options are not given, determine from UI
if isempty(options)
    try
        options = startOptions();
    catch ME
        warning(ME.message);
        return
    end
end

% check if Scenario object is given as input
scenario = read_object_from_input(varargin, 'Scenario');
if isempty(scenario)
    random_seed = RandStream('mt19937ar');
    scenario = create_scenario(options, random_seed);
end

% write scenario to disk if distributed (for lab or local debugging with main_distributed())
if scenario.options.isPB == true
    save('scenario.mat','scenario');
end
if scenario.options.isPB && scenario.options.is_sim_lab == false && scenario.options.isParl
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
    factory = HLCFactory();
    factory.set_scenario(scenario);
    if scenario.options.isPB == true && scenario.options.isParl
        %% simulate distribution locally using the Parallel Computing Toolbox
        get_parallel_pool(scenario.options.amount);
        plot = scenario.options.visu(1);
        if plot
            factory.set_visualization_data_queue;
            % create central plotter - used by all workers via data queue
            plotter = PlotterOnline(factory.scenario);
            afterEach(factory.visualization_data_queue, @plotter.data_queue_callback);
        end
        spmd(scenario.options.amount)
            hlc = factory.get_hlc(scenario.options.veh_ids(labindex));
            [result,scenario] = hlc.run();
        end
        if plot
            plotter.close_figure();
        end
        result={result{:}};
        scenario={scenario{:}};
    else
        hlc = factory.get_hlc(scenario.options.veh_ids);
        [result,scenario] = hlc.run();
    end
end
end