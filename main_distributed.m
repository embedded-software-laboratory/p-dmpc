function [result, scenario] = main_distributed(vehicle_id)

    %startup();

    if verLessThan('matlab', '9.12')
        warning("Code is developed in MATLAB 2022a, prepare for backward incompatibilities.")
    end

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    % get HLC
    factory = HLCFactory();
    factory.set_scenario(scenario);
    dry_run = (scenario.options.environment == Environment.CPMLab); % TODO: Use dry run also for unified lab api?

    if scenario.options.is_prioritized == true
        hlc = factory.get_hlc(vehicle_id, dry_run);
        [result, scenario] = hlc.run();
    else
        warning("Use main_distributed.m only for pb-scenarios.")
    end

end
