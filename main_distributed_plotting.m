function main_distributed_plotting()
    scenario = load('scenario.mat', 'scenario').scenario;
    plotter = PlotterOnline(scenario);
    generate_plotting_info_msgs();
    ros2_node = ros2node('/plant_plotting');
    veh_ids = scenario.options.veh_ids;
    amount = scenario.options.amount;
    subscribers = cell(amount, 1);

    for i_veh = 1:amount
        disp(['init subscriber for vehicle ', num2str(veh_ids(i_veh))])
        subscribers{i_veh} = create_subscriber_plotting(ros2_node, plotter, veh_ids(i_veh));
    end

    while true
        pause(0.5)
    end

end

function subscriber = create_subscriber_plotting(ros2_node, plotter, vehicle_id)
    options = struct("History", "keeplast", "Depth", 40, "Reliability", "reliable", "Durability", "transientlocal");
    topic_name_subscribe = ['/plant_', num2str(vehicle_id), '_plotting'];
    subscriber = ros2subscriber(ros2_node, topic_name_subscribe, "plotting_info/PlottingInfo", @plotter.ros2_callback, options);
end

function generate_plotting_info_msgs()
    msgList = ros2("msg", "list"); % get all ROS 2 message types

    if ((sum(cellfun(@(c)strcmp(c, 'plotting_info/PlottingInfo'), msgList)) == 0))
        [file_path, ~, ~] = fileparts(mfilename('fullpath'));
        disp('Generating ROS 2 custom message type for distributed plotting...')

        try
            ros2genmsg([file_path,filesep,'plant']);
        catch ME
            disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
                  'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
            throw(ME)
        end

    else
        disp(['No generation of ROS 2 custom message type for distributed plotting, since at least "plotting_info/PlottingInfo" ' ...
              'message exists. If message types are missing regenerate all by removing the folder matlab_msg_gen...'])
    end

end
