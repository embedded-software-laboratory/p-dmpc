classdef (Abstract) Plant < handle
    % INTERFACEEXPERIMENT    Abstract class used for defining properties and methods used by different experiment setups.

    properties (Access = protected)
        % struct used for every iteration
        cur_node (:, :) double % (n_veh, NodeInfo.n_cols) created by node.m

        % measurement used for every iteration
        measurements (:, 1) PlantMeasurement % (n_veh, 1)

        dt_seconds (1, 1) double
        amount (1, 1) double

        % used by labs
        Hp (1, 1) double
        manual_control_config ManualControlConfig % (1, 1)

        % used by simulation
        options_plot_online OptionsPlotOnline % (1, 1)

    end

    properties (GetAccess = public, SetAccess = private)
        % public so that the HLC can access them
        % all active vehicle ids. when running distributedly, these can differ from controlled ids
        all_vehicle_ids (1, :) uint8 % uint8 to conform to ids sent by the middleware
        % which vehicles will controlled by this experiment instance
        controlled_vehicle_ids (1, :) uint8
    end

    properties (Dependent, GetAccess = public)
        indices_in_vehicle_list
    end

    methods (Abstract)
        [cav_measurements, hdv_measurements] = measure(obj, ~)
        apply(obj, info, experiment_result, k, mpa)
        got_stop = is_stop(obj)
        end_run(obj)
    end

    methods
        % get methods for dependent properties
        function indices = get.indices_in_vehicle_list(obj)
            [~, indices] = ismember(obj.controlled_vehicle_ids, obj.all_vehicle_ids);
        end

    end

    methods (Access = public)

        function obj = Plant()
        end

        function setup(obj, options, scenario, all_vehicle_ids, controlled_vehicle_ids)

            arguments
                obj (1, 1) Plant
                options (1, 1) Config
                scenario (1, 1) Scenario
                all_vehicle_ids (1, :) uint8
                controlled_vehicle_ids (1, :) uint8
            end

            % This function does everything in order to run the object
            % later on. If further initialization needs to be done this
            % method shall be overriden and called in a child class.

            % save options that are required as properties in subclasses
            obj.dt_seconds = options.dt_seconds;
            obj.amount = options.amount;
            obj.Hp = options.Hp;
            obj.manual_control_config = options.manual_control_config;
            obj.options_plot_online = options.options_plot_online;

            % save vehicle_ids as properties
            obj.controlled_vehicle_ids = controlled_vehicle_ids;
            obj.all_vehicle_ids = all_vehicle_ids;

            % all vehicles have the same initial speed and steering
            initial_speed = 0;
            initial_steering = 0;

            % construct measurements
            obj.measurements(options.amount, 1) = PlantMeasurement();

            % set initial vehicle measurements
            for i_vehicle = 1:options.amount
                obj.measurements(i_vehicle) = PlantMeasurement( ...
                    scenario.vehicles(i_vehicle).x_start, ...
                    scenario.vehicles(i_vehicle).y_start, ...
                    scenario.vehicles(i_vehicle).yaw_start, ...
                    initial_speed, ...
                    initial_steering ...
                );
            end

            % temporarily create an MPA to get vehicles` trim config
            tmp_mpa = MotionPrimitiveAutomaton(scenario.model, options);

            % find initial trim from mpa (equal for all vehicles)
            initial_trim = find([tmp_mpa.trims.speed] == 0 & [tmp_mpa.trims.steering] == 0, 1);
            initial_trims = repmat(initial_trim, 1, options.amount);

            % set initial node with information from scenario
            obj.cur_node = node( ...
                0, ...
                initial_trims', ...
                [scenario.vehicles(:).x_start]', ...
                [scenario.vehicles(:).y_start]', ...
                [scenario.vehicles(:).yaw_start]', ...
                zeros(options.amount, 1), ...
                zeros(options.amount, 1) ...
            );

        end

        function receive_map(~)
            % InterfaceExperiments may allow to retrieve a lab specific
            % map. If so, this function needs to be overriden. By default
            % this function is not available.
            error('This interface does not provide the possibility to retrieve a lab specific map.');
        end

        function dt_seconds = get_step_time(obj)
            % return step time that is set by the plant
            dt_seconds = obj.dt_seconds;
        end

        function synchronize_start_with_plant(obj)
            % can be overriden in child class if needed
        end

    end

    methods (Access = protected)

        function [x0, trim_indices] = measure_node(obj, mpa)
            speeds = zeros(obj.amount, 1);

            for iVeh = 1:obj.indices_in_vehicle_list
                speeds(iVeh) = mpa.trims(obj.cur_node(iVeh, NodeInfo.trim)).speed;
            end

            x0 = [obj.cur_node(:, NodeInfo.x), obj.cur_node(:, NodeInfo.y), obj.cur_node(:, NodeInfo.yaw), speeds];
            trim_indices = obj.cur_node(:, NodeInfo.trim);
        end

    end

end
