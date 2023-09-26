classdef (Abstract) Plant < handle
    % INTERFACEEXPERIMENT    Abstract class used for defining properties and methods used by different experiment setups.

    properties (Access = protected)
        % struct used for every iteration
        x0
        trim_indices
        cur_node
        scenario
        k
    end

    properties (GetAccess = public, SetAccess = private)
        % public so that the HLC can access them
        % all active vehicle ids. when running distributedly, these can differ from controlled ids
        all_vehicle_ids (1, :) uint8 % uint8 to conform to ids sent by the middleware
        % which vehicles will controlled by this experiment instance
        controlled_vehicle_ids (1, :) uint8
    end

    properties (Dependent, GetAccess = public)
        amount % amount of vehicles controlled by this experiment instance
        indices_in_vehicle_list
    end

    methods (Abstract)
        [x0, trim_indices] = measure(obj, ~)
        apply(obj, info)
        got_stop = is_stop(obj)
        end_run(obj)
    end

    methods
        % get methods for dependent properties
        function indices = get.indices_in_vehicle_list(obj)
            [~, indices] = ismember(obj.controlled_vehicle_ids, obj.all_vehicle_ids);
        end

        function amount = get.amount(obj)
            amount = length(obj.controlled_vehicle_ids);
        end

    end

    methods (Access = public)

        function obj = Plant()
        end

        function setup(obj, scenario, all_vehicle_ids, controlled_vehicle_ids)

            arguments
                obj (1, 1) Plant
                scenario (1, 1) Scenario
                all_vehicle_ids (1, :) uint8
                controlled_vehicle_ids (1, :) uint8
            end

            % This function does everything in order to run the object
            % later on. If further initialization needs to be done this
            % method shall be overriden and called in a child class.
            obj.scenario = scenario;

            obj.controlled_vehicle_ids = controlled_vehicle_ids;
            obj.all_vehicle_ids = all_vehicle_ids;

            % temporarily create an MPA to get vehicles` trim config
            tmp_mpa = MotionPrimitiveAutomaton(scenario.model, scenario.options);
            initial_state = find([tmp_mpa.trims.speed] == 0 & [tmp_mpa.trims.steering] == 0, 1);

            for iVeh = 1:scenario.options.amount
                % initialize vehicle ids of all vehicles
                obj.scenario.vehicles(iVeh).trim_config = initial_state;

            end

            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.options.amount, 1), zeros(obj.scenario.options.amount, 1));

        end

        function receive_map(~)
            % InterfaceExperiments may allow to retrieve a lab specific
            % map. If so, this function needs to be overriden. By default
            % this function is not available.
            error('This interface does not provide the possibility to retrieve a lab specific map.');
        end

        function synchronize_start_with_plant(obj)
            % can be overriden in child class if needed
        end

    end

    methods (Access = protected)

        function [x0, trim_indices] = measure_node(obj, mpa)
            speeds = zeros(obj.scenario.options.amount, 1);

            for iVeh = 1:obj.indices_in_vehicle_list
                speeds(iVeh) = mpa.trims(obj.cur_node(iVeh, NodeInfo.trim)).speed;
            end

            x0 = [obj.cur_node(:, NodeInfo.x), obj.cur_node(:, NodeInfo.y), obj.cur_node(:, NodeInfo.yaw), speeds];
            trim_indices = obj.cur_node(:, NodeInfo.trim);
        end

    end

end
