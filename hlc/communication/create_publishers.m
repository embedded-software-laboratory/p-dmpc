function create_publishers(hlc)
    % creates ros2 publisher for the given high level controller
    % for each controlled vehicle a ros2 node is created
    % all vehicles share topics for related messages
    %
    % INPUT:
    %   hlc: handle of high level controller
    %

    timer_nodes = tic;
    disp('Creating ROS 2 nodes...');

    for vehicle_index = hlc.plant.indices_in_vehicle_list
        vehicle_id = hlc.plant.all_vehicle_ids(vehicle_index);
        % create instance of the communication class
        hlc.traffic_communication{vehicle_index} = TrafficCommunication();
        % create node and store topic name and message type
        hlc.traffic_communication{vehicle_index}.initialize( ...
            vehicle_id, ...
            'traffic', ...
            '/vehicle_traffic', ...
            'veh_msgs/Traffic' ...
        );

        % create instance of the communication class
        hlc.predictions_communication{vehicle_index} = PredictionsCommunication();
        % create node and store topic name and message type
        hlc.predictions_communication{vehicle_index}.initialize( ...
            vehicle_id, ...
            'prediction', ...
            '/vehicle_prediction', ...
            'veh_msgs/Predictions' ...
        );
    end

    duration_nodes = toc(timer_nodes);
    disp(['Finished creating nodes in ', ...
              num2str(duration_nodes), ' seconds.']);

    timer_publisher = tic;
    disp('Creating ROS 2 publishers...');

    for vehicle_index = hlc.plant.indices_in_vehicle_list
        % create publishers
        hlc.traffic_communication{vehicle_index}.create_publisher();
        hlc.predictions_communication{vehicle_index}.create_publisher();
    end

    duration_publisher = toc(timer_publisher);
    disp(['Finished creating publishers in ', ...
              num2str(duration_publisher), ' seconds.']);

end
