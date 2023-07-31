classdef (Abstract) Plant < handle
    % INTERFACEEXPERIMENT    Abstract class used for defining properties and methods used by different experiment setups.

    properties (Access = protected)
        % struct used for every iteration
        x0
        trim_indices
        cur_node
        scenario
        k
        amount
    end

    properties (Access = public)
        % public so that the HLC can access them
        indices_in_vehicle_list
        veh_ids % which vehicles will controlled by this experiment instance
        all_veh_ids % all active vehicle ids. when running distributedly, these can differ from veh_ids
    end

    methods (Abstract)
        [x0, trim_indices] = measure(obj, ~)
        apply(obj, info)
        got_stop = is_stop(obj)
        end_run(obj)
    end

    methods

        function obj = Plant()
        end

        function set_to_control_single_vehicle(obj, vehicle_id)

            arguments
                obj (1, 1) Plant
                vehicle_id (1, 1)
            end

            obj.amount = 1;
            obj.veh_ids = vehicle_id;
            obj.indices_in_vehicle_list = find(obj.all_veh_ids == vehicle_id, 1);

            disp('during set_to_control_single_vehicle, have veh_ids:');
            disp(obj.veh_ids);
            disp('all_veh_ids:');
            disp(obj.all_veh_ids);
            disp('and indices:');
            disp(obj.indices_in_vehicle_list);
        end

        function setup(obj, scenario, vehicle_ids)

            arguments
                obj (1, 1) Plant
                scenario (1, 1) Scenario
                vehicle_ids (1, :)
            end

            % This function does everything in order to run the object
            % later on. If further initialization needs to be done this
            % method shall be overriden and called in a child class.
            obj.scenario = scenario;

            obj.amount = length(vehicle_ids);
            obj.veh_ids = vehicle_ids;
            obj.all_veh_ids = vehicle_ids;
            obj.indices_in_vehicle_list = 1:obj.amount;

            disp('after Plant.setup, have veh_ids, amount');
            disp(obj.veh_ids);
            disp(obj.amount);
            disp('and indices:');
            disp(obj.indices_in_vehicle_list);
            disp('as well as all_veh_ids:');

            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.options.amount, 1), zeros(obj.scenario.options.amount, 1));

        end

        function [x0, trim_indices] = measure_node(obj)
            % take last planned state as new actual state
            speeds = zeros(obj.scenario.options.amount, 1);

            for iVeh = 1:obj.indices_in_vehicle_list
                speeds(iVeh) = obj.scenario.mpa.trims(obj.cur_node(iVeh, NodeInfo.trim)).speed;
            end

            x0 = [obj.cur_node(:, NodeInfo.x), obj.cur_node(:, NodeInfo.y), obj.cur_node(:, NodeInfo.yaw), speeds];
            trim_indices = obj.cur_node(:, NodeInfo.trim);
        end

        function receive_map(~)
            % InterfaceExperiments may allow to retrieve a lab specific
            % map. If so, this function needs to be overriden. By default
            % this function is not available.
            error('This interface does not provide the possibility to retrieve a lab specific map.');
        end

        function send_ready_msg(obj)
            % can be overriden in child class if needed
        end

    end

end
