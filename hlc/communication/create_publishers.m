function create_publishers(hlc)
    % create_publishers This function creates all ROS publishers for the given hlc.
    % ROS 2 nodes are created for each vehicle. Each vehicle has its own topic
    % and sends its data only to its own topic.
    % As a precondition, generate_ros2_msgs() must have been called before.
    %
    % INPUT:
    %   hlc: Controller implementing the HighLevelController
    %

    %     topicList = ros2("topic","list");
    %     nodeList = ros2("node","list");
    if isempty(hlc.scenario.vehicles(hlc.plant.indices_in_vehicle_list(1)).communicate)
        start = tic;
        disp('Creating ROS 2 publishers...')

        for vehicle_id = hlc.plant.controlled_vehicle_ids
            index = find(vehicle_id == hlc.plant.all_vehicle_ids);
            hlc.scenario.vehicles(index).communicate.traffic = TrafficCommunication(); % create instance of the Comunication class
            hlc.scenario.vehicles(index).communicate.traffic = initialize_communication(hlc.scenario.vehicles(index).communicate.traffic, vehicle_id); % initialize
            hlc.scenario.vehicles(index).communicate.traffic = create_publisher(hlc.scenario.vehicles(index).communicate.traffic); % create publisher

            hlc.scenario.vehicles(index).communicate.predictions = PredictionsCommunication(); % create instance of the Comunication class
            hlc.scenario.vehicles(index).communicate.predictions = initialize_communication(hlc.scenario.vehicles(index).communicate.predictions, vehicle_id); % initialize
            hlc.scenario.vehicles(index).communicate.predictions = create_publisher(hlc.scenario.vehicles(index).communicate.predictions); % create publisher
        end

        duration = toc(start);
        disp(['Finished creating publishers in ' num2str(duration) ' seconds.'])
    end

end
