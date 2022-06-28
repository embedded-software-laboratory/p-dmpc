function result = run_simulation_if_scenario_changed(scenario, doOnlinePlot, doPlotExploration)
result = get_result_struct(scenario);
output_path = fullfile(result.output_path, 'data.mat');
% TODO Add replay function, then no new simulation necessary
if (isfile(output_path) && ~doOnlinePlot)
    result_stored = load(output_path);
    result_stored = result_stored.result;
    if (isequal(result_stored.scenario, scenario))
        result = result_stored;
        return
    end
end
result = run_simulation(scenario, doOnlinePlot, doPlotExploration);
end