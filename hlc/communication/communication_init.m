function communication_init(obj)
    % COMMUNICATION_INIT This function initializes the communication network
    % by sending initial messages. This function also waits for other obj
    % running on other machines or in other processes on the same machine (synchronization)
    %
    % INPUT:
    %   obj: handle of prioritized controller
    %

    % measure vehicles' initial poses and trims
    [x0_measured, trims_measured] = obj.plant.measure(obj.mpa);

    Hp = obj.scenario.options.Hp;

    %% send initial message such that subscriber isn't empty during first controller time step.
    %% Also used for initial synchronization for distributed runs

    % Communicate predicted trims, pridicted lanelets and areas to other vehicles
    for vehicle_index = obj.plant.indices_in_vehicle_list
        predicted_trims = repmat(trims_measured(vehicle_index), 1, Hp + 1); % current trim and predicted trims in the prediction horizon

        obj.iter.x0(vehicle_index, :) = x0_measured(vehicle_index, :);
        % get own trim
        obj.iter.trim_indices(vehicle_index) = trims_measured(vehicle_index);
        x0 = obj.iter.x0(vehicle_index, indices().x);
        y0 = obj.iter.x0(vehicle_index, indices().y);
        heading = obj.iter.x0(vehicle_index, indices().heading);
        speed = obj.iter.x0(vehicle_index, indices().speed);
        current_pose = [x0, y0, heading, speed];

        predicted_lanelets = get_predicted_lanelets(obj.scenario, obj.mpa, obj.iter, vehicle_index, x0, y0);

        % get vehicles currently occupied area
        x_rec1 = [-1, -1, 1, 1, -1] * (obj.scenario.vehicles(vehicle_index).Length / 2 + obj.scenario.options.offset); % repeat the first entry to enclose the shape
        y_rec1 = [-1, 1, 1, -1, -1] * (obj.scenario.vehicles(vehicle_index).Width / 2 + obj.scenario.options.offset);
        % calculate displacement of model shape
        [x_rec2, y_rec2] = translate_global(heading, x0, y0, x_rec1, y_rec1);
        occupied_area.normal_offset = [x_rec2; y_rec2];

        x_rec1_without_offset = [-1, -1, 1, 1, -1] * (obj.scenario.vehicles(vehicle_index).Length / 2); % repeat the first entry to enclose the shape
        y_rec1_without_offset = [-1, 1, 1, -1, -1] * (obj.scenario.vehicles(vehicle_index).Width / 2);
        [x_rec2_without_offset, y_rec2_without_offset] = translate_global(heading, x0, y0, x_rec1_without_offset, y_rec1_without_offset);
        occupied_area.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

        predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
        reachable_sets = {}; % for initial time step, the reachable sets are not computed yet

        obj.traffic_communication{vehicle_index}.send_message( ...
            obj.k, ...
            current_pose, ...
            predicted_trims(1), ...
            predicted_lanelets, ...
            occupied_area, ...
            reachable_sets ...
        );
        obj.predictions_communication{vehicle_index}.send_message( ...
            obj.k, ...
            predicted_occupied_areas ...
        );
    end

    % read from all other vehicles to make sure all vehicles are ready (synchronization)
    for vehicle_index = obj.plant.indices_in_vehicle_list
        % loop over vehicles that read messages
        other_vehicles = setdiff(1:obj.scenario.options.amount, vehicle_index);

        for vehicle_index_subscribed = other_vehicles
            % loop over vehicles/hlcs that are subscribed
            obj.traffic_communication{vehicle_index}.read_message( ...
                obj.plant.all_vehicle_ids(vehicle_index_subscribed), ...
                obj.k, ...
                true, ...
                40.0 ...
            );
            obj.predictions_communication{vehicle_index}.read_message( ...
                obj.plant.all_vehicle_ids(vehicle_index_subscribed), ...
                obj.k, ...
                true, ...
                40.0 ...
            );
        end

    end

    disp('communication initialized');

end
