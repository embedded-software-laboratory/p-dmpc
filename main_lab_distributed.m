function [result,scenario] = main_lab_distributed(vehicle_id)    

if verLessThan('matlab','9.12')
    warning("Code is developed in MATLAB 2022a, prepare for backward incompatibilities.")
end

% read scenario from disk
scenario = load('scenario.mat', 'scenario').scenario;

% get HLC
factory = HLCFactory();
factory.set_scenario(scenario);
if scenario.options.isPB == true
    plot = scenario.options.visu(1);
    if plot
        warning("Plotting doesn't work when using distributed deployment")
    end
    hlc = factory.get_hlc(vehicle_id);
    [result,scenario] = hlc.run();
else
    warning("Use main_lab_distributed.m only for pb-scenarios.")
end
end