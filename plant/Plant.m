classdef (Abstract) Plant < handle
    % INTERFACEEXPERIMENT    Abstract class used for defining properties and methods used by different experiment setups.

    properties (Access = protected)
        % struct used for every iteration
        x0
        trim_indices
        cur_node
        scenario
        k
        veh_ids % which vehicles will controlled by this experiment instance
        amount
        indices_in_vehicle_list
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

        function set_vehicle_ids(obj, vehicle_ids)
            obj.veh_ids = vehicle_ids;
            obj.amount = length(obj.veh_ids);

            if obj.amount == 1
                obj.indices_in_vehicle_list = [find(obj.scenario.options.veh_ids == obj.veh_ids(1), 1)];
            else
                obj.indices_in_vehicle_list = 1:obj.amount;
            end

        end

        function setup(obj, scenario)
            % This function does everything in order to run the object
            % later on. If further initialization needs to be done this
            % method shall be overriden and called in a child class.
            obj.scenario = scenario;

            % The child class method should have set the vehicle ids
            % via scenario.set_vehicle_ids
            assert(~isempty(obj.scenario.options.veh_ids));

            obj.set_vehicle_ids(obj.scenario.options.veh_ids);

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
