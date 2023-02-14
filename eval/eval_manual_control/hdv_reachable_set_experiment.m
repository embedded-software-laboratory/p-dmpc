function result = hdv_reachable_set_experiment()
    options = Config();
    options.mixed_traffic_config = Mixed_traffic_config();
    options.trim_set = 4;
    options.T_end = 1;
    options.Hp = 6;
    options.isPB = true;
    options.isParl = false;
    options.is_sim_lab = true;
    options.visu = [true, false];
    options.isSaveResult = 1;
    options.isSaveResultReduced = 0;
    options.bound_reachable_sets = false;

    % visualization for video
    options.optionsPlotOnline.isShowCoupling = false;

    vehicle_ids = [8, 9];

    options.amount = length(vehicle_ids);
    options.scenario_name = 'Commonroad';
    options.veh_ids = vehicle_ids;

    scenario = create_scenario(options);
    scenario.vehicles(1).x_start = 2;
    scenario.vehicles(1).y_start = 3.89;
    scenario.vehicles(1).yaw_start = 0;

    result = main(scenario, options);
    hdv_reachable_set_plot(result);
    cav_occupied_area_plot(result);
end
