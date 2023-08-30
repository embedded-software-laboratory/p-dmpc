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

        function setup(obj, scenario, veh_ids)
            % This function does everything in order to run the object
            % later on. If further initialization needs to be done this
            % method shall be overriden and called in a child class.
            obj.scenario = scenario;
            obj.veh_ids = veh_ids;
            obj.amount = length(veh_ids);

            if obj.amount == 1
                obj.indices_in_vehicle_list = [find(obj.scenario.options.veh_ids == obj.veh_ids(1), 1)];
            else
                obj.indices_in_vehicle_list = 1:obj.amount;
            end

            % temporarily create an MPA to get vehicles` trim config
            tmp_mpa = MotionPrimitiveAutomaton(scenario.model, scenario.options);
            initial_state = find([tmp_mpa.trims.speed] == 0 & [tmp_mpa.trims.steering] == 0, 1);

            for iVeh = 1:scenario.options.amount
                % initialize vehicle ids of all vehicles
                obj.scenario.vehicles(iVeh).trim_config = initial_state;

            end

            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.options.amount, 1), zeros(obj.scenario.options.amount, 1));
        end

        function [x0, trim_indices] = measure_node(obj, mpa)
            % take last planned state as new actual state
            speeds = zeros(obj.scenario.options.amount, 1);

            for iVeh = 1:obj.indices_in_vehicle_list
                speeds(iVeh) = mpa.trims(obj.cur_node(iVeh, NodeInfo.trim)).speed;
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
            % placeholder to be overwritten if needed
        end

    end

end
