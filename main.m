function [result,scenario,options] = main(varargin)
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
    % write scenario to disk if distributed
    if scenario.options.isPB == true && scenario.options.is_sim_lab == false
        save('scenario.mat','scenario');
        disp('Scenario was written to disk. Select run_scenario_distributed in LCC next.')

    else
        factory = HLCFactory();
        factory.set_scenario(scenario);
        if scenario.options.isPB == true
        %if false
            get_parallel_pool(scenario.options.amount);
            spmd(1)
                hlc = factory.get_hlc(scenario.options.veh_ids);
                [result,scenario] = hlc.run();
            end
        else
            hlc = factory.get_hlc(scenario.options.veh_ids);
            [result,scenario] = hlc.run();
        end
    end
end
end