function scenario = create_scenario(options, random_seed)
%create scenario from options

%% options preprocessing
% Use options to setup scenario
manualVehicle_id = 0;
manualVehicle_id2 = 0;
options.max_num_CLs = min(options.max_num_CLs, options.amount);
if options.is_sim_lab
    disp('Running in MATLAB simulation...')
    if isempty(options.veh_ids)
        switch options.amount
            % specify vehicles IDs
            case 2
                vehicle_ids = [16,18];
            case 4
                vehicle_ids = [14,16,18,20];
    %       case 6
    %           vehicle_ids = [10,14,16,17,18,20];
            otherwise
    %           vehicle_ids = 1:options.amount; % default IDs
                if options.max_num_CLs == 1
                    % if allowed computation is only 1, the first 8
                    % vehicles will not be used to avoid infeasibility at
                    % the first time step as there may be vehicles being
                    % very colse to others
                    vehicle_ids = sort(randsample(random_seed,9:40,options.amount),'ascend');
                else
                    vehicle_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
                end
        end
    else
        vehicle_ids = options.veh_ids;
    end
    options.veh_ids = sort(vehicle_ids);
    options.is_mixed_traffic = 0;
else
    disp('Running in CPM Lab...')
    vehicle_ids = options.veh_ids;
    options.isPB = true;

    if options.is_mixed_traffic

        manualVehicle_id = options.manualVehicle_id;

        if ~strcmp(manualVehicle_id, 'No MV')
            manualVehicle_id = str2double(options.manualVehicle_id);
            options.mixed_traffic_config.first_manual_vehicle_mode = str2double(options.mixed_traffic_config.first_manual_vehicle_mode);

            if ~strcmp(options.manualVehicle_id2, 'No second MV')
                manualVehicle_id2 = str2double(options.manualVehicle_id2);
                options.mixed_traffic_config.second_manual_vehicle_mode = str2double(ooptions.mixed_traffic_config.second_manual_vehicle_mode);
            end
        else
            manualVehicle_id = 0;
        end

        if options.collisionAvoidanceMode == 1
            options.priority = 'right_of_way_priority';
        elseif options.collisionAvoidanceMode == 2
            options.priority = 'right_of_way_priority';
        else
            options.priority = 'mixed_traffic_priority';
            options.visualize_reachable_set = true;
        end
    end
    options.veh_ids = sort(vehicle_ids);
end
if  ~options.is_mixed_traffic
    options.mixed_traffic_config.first_manual_vehicle_id = 0;
    options.mixed_traffic_config.second_manual_vehicle_id = 0;
end

assert(length(options.veh_ids) == options.amount);

%% Setup Scenario
switch options.scenario_name
    case 'Circle_scenario'
        scenario = circle_scenario(options);
    case 'Commonroad'
        scenario = commonroad(options, vehicle_ids, manualVehicle_id, manualVehicle_id2, options.is_sim_lab);
end

for iVeh = 1:options.amount
    % initialize vehicle ids of all vehicles
    scenario.vehicles(iVeh).ID = scenario.options.veh_ids(iVeh);
end
