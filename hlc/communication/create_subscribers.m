function create_subscribers(hlc)
    % create_subscribers This function creates the ROS subscribers for the given hlc.
    % ROS 2 nodes are created for each vehicle. Each vehicle has its own topic
    % and sends its data only to its own topic.
    %
    % INPUT:
    %   hlc: Controller implementing the HighLevelController
    %

    % Create subscribers.
    % Each vehicle subscribes all other vehicles.
    % NOTE that subscribers are create only once but not loopover all
    % vehicles to let all of them subscribe others because it is
    % time-consuming to create many subscribers.
    % The subscribers will be used by the given HLC.
    disp('Creating ROS 2 subscribers...')
    start = tic;

    for vehicle_index = hlc.plant.indices_in_vehicle_list
        % loop over vehicles that create subscriber
        veh_indices_to_be_subscribed = setdiff(1:hlc.scenario.options.amount, vehicle_index);
        veh_ids_to_be_subscribed = [hlc.plant.all_vehicle_ids(veh_indices_to_be_subscribed)];
        hlc.traffic_communication{vehicle_index}.create_subscriber(veh_indices_to_be_subscribed, veh_ids_to_be_subscribed, hlc.scenario.options.amount);
        hlc.scenario.vehicles(vehicle_index).communicate.predictions.create_subscriber(veh_indices_to_be_subscribed, veh_ids_to_be_subscribed, hlc.scenario.options.amount);
    end

    if length(hlc.plant.indices_in_vehicle_list) == 1
        pause(5.0) % wait for all subscribers to be created in distributed case, because otherwise early sent messages will be lost.
    end

    duration = toc(start);
    disp(['Finished creating subscribers in ' num2str(duration) ' seconds.'])
end
