function create_subscribers(hlc)
    % creates ros2 subscribers for the given high level controller
    % all vehicles share topics for related messages
    %
    % INPUT:
    %   hlc: handle of high level controller
    %

    disp('Creating ROS 2 subscribers...')
    start = tic;

    for vehicle_index = hlc.plant.indices_in_vehicle_list
        % loop over vehicles that create subscriber
        hlc.traffic_communication{vehicle_index}.create_subscriber();
        hlc.predictions_communication{vehicle_index}.create_subscriber();
    end

    if length(hlc.plant.indices_in_vehicle_list) == 1
        pause(5.0) % wait for all subscribers to be created in distributed case, because otherwise early sent messages will be lost.
    end

    duration = toc(start);
    disp(['Finished creating subscribers in ' num2str(duration) ' seconds.'])
end
