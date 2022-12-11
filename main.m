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
        if scenario.options.isParl == true
            save('scenario.mat','scenario');
            if scenario.options.is_sim_lab == false
                disp('Scenario was written to disk. Select run_scenario_distributed in LCC next.')
            end
        end
    end


    hlc = HLC();
    hlc.setScenario(scenario);
    hlc.getHlc();
    hlc.run();

    % run scenario
    [result,scenario] = run_scenario(scenario);
end
