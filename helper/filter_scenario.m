function scenario = filter_scenario(scenario, vehicle_filter)
    scenario.vehicles = scenario.vehicles(vehicle_filter);
    scenario.options.amount = sum(vehicle_filter);
end
