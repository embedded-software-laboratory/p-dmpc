function = main_distributed_plotting()
    scenario = load('scenario.mat', 'scenario').scenario;
    plotter = PlotterOnline(scenario);
    ros2_node = ros2node('/plant_plotting');
    veh_ids = scenario.options.veh_ids;
    amount = scenario.options.amount;
    subscribers = cell(amount, 1);

    for i_veh = 1:amount
        subscribers{i_veh} = create_subscriber_plotting(ros2_node, veh_ids(i_veh));
    end

    while true
    end

end

function subscriber = create_subscriber_plotting(ros2_node, vehicle_id)
    options = struct("History", "keeplast", "Depth", 40, "Reliability", "reliable", "Durability", "transientlocal");
    topice_name_subscribe = ['/plant_', num2str(vehicle_id), '_plotting'];
    subscriber = ros2publisher(obj.ros2_node, topic_name_subscribe, "plotting_info/PlottingInfo", @plotter.ros2_callback, options);
end
