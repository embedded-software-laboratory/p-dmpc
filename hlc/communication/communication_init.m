function communication_init(hlc)
    % COMMUNICATION_INIT This function initializes the communication network
    % by sending initial messages. This function also waits for other hlc
    % running on other machines or in other processes on the same machine (synchronization)
    %
    % INPUT:
    %   hlc: Controller implementing the HighLevelController
    %

    % measure vehicles' initial poses and trims
    [x0_measured, trims_measured] = hlc.plant.measure();

    Hp = hlc.scenario.options.Hp;

    %% send initial message such that subscriber isn't empty during first controller time step.
    %% Also used for initial synchronization for distributed runs
    if ~hlc.scenario.options.is_manual_control
        % Communicate predicted trims, pridicted lanelets and areas to other vehicles
        for veh_index = hlc.indices_in_vehicle_list
            predicted_trims = repmat(trims_measured(veh_index), 1, Hp + 1); % current trim and predicted trims in the prediction horizon

            hlc.iter.x0(veh_index, :) = x0_measured(veh_index, :);
            % get own trim
            hlc.iter.trim_indices(veh_index) = trims_measured(veh_index);
            x0 = hlc.iter.x0(veh_index, indices().x);
            y0 = hlc.iter.x0(veh_index, indices().y);
            heading = hlc.iter.x0(veh_index, indices().heading);
            speed = hlc.iter.x0(veh_index, indices().speed);
            current_pose = [x0, y0, heading, speed];

            predicted_lanelets = get_predicted_lanelets(hlc.scenario, hlc.iter, veh_index, x0, y0);

            % get vehicles currently occupied area
            x_rec1 = [-1, -1, 1, 1, -1] * (hlc.scenario.vehicles(veh_index).Length / 2 + hlc.scenario.options.offset); % repeat the first entry to enclose the shape
            y_rec1 = [-1, 1, 1, -1, -1] * (hlc.scenario.vehicles(veh_index).Width / 2 + hlc.scenario.options.offset);
            % calculate displacement of model shape
            [x_rec2, y_rec2] = translate_global(heading, x0, y0, x_rec1, y_rec1);
            occupied_area.normal_offset = [x_rec2; y_rec2];

            x_rec1_without_offset = [-1, -1, 1, 1, -1] * (hlc.scenario.vehicles(veh_index).Length / 2); % repeat the first entry to enclose the shape
            y_rec1_without_offset = [-1, 1, 1, -1, -1] * (hlc.scenario.vehicles(veh_index).Width / 2);
            [x_rec2_without_offset, y_rec2_without_offset] = translate_global(heading, x0, y0, x_rec1_without_offset, y_rec1_without_offset);
            occupied_area.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

            predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
            reachable_sets = {}; % for initial time step, the reachable sets are not computed yet
            hlc.scenario.vehicles(veh_index).communicate.predictions.send_message(hlc.k, predicted_occupied_areas);
            hlc.scenario.vehicles(veh_index).communicate.traffic.send_message(hlc.k, current_pose, predicted_trims(1), predicted_lanelets, occupied_area, reachable_sets);
        end

        % read from all other vehicles to make sure all vehicles are ready (synchronization)
        if length(hlc.indices_in_vehicle_list) ~= 1
            other_vehicles = 1:hlc.scenario.options.amount; % still read from own publishers to make sure messages are arriving
        else
            other_vehicles = setdiff(1:hlc.scenario.options.amount, hlc.indices_in_vehicle_list);
        end

        for veh_index = other_vehicles
            disp(['reading initial msg from vehicle ', num2str(veh_index)]);
            read_message(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate.traffic, hlc.ros_subscribers.traffic{veh_index}, hlc.k, true, 40.0);
            read_message(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate.predictions, hlc.ros_subscribers.predictions{veh_index}, hlc.k, true, 40.0);
        end

        disp('communication initialized');
    end

end
