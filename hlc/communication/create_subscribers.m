function create_subscribers(obj)
    % creates ros2 subscribers for the given high level controller
    % all vehicles share topics for related messages
    %
    % INPUT:
    %   obj: handle of prioritized controller
    %

    disp('Creating ROS 2 subscribers...')
    start = tic;

    for vehicle_index = obj.plant.indices_in_vehicle_list
        % loop over vehicles that create subscriber
        obj.traffic_communication{vehicle_index}.create_subscriber();
        obj.predictions_communication{vehicle_index}.create_subscriber();
    end

    if length(obj.plant.indices_in_vehicle_list) == 1
        pause(5.0) % wait for all subscribers to be created in distributed case, because otherwise early sent messages will be lost.
    end

    duration = toc(start);
    disp(['Finished creating subscribers in ' num2str(duration) ' seconds.'])
end
