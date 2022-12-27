function [result,scenario,options] = main(varargin)
    % MAIN  main function for graph-based receeding horizon control
    
    if verLessThan('matlab','9.10')
        warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
    end


    % check if OptionsMain object is given as input
    options = read_object_from_input(varargin, 'Config');
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


    % run scenario
    [result,scenario] = run_scenario(scenario);
end