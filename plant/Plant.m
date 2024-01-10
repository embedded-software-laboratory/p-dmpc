classdef (Abstract) Plant < handle
    % Plant - Abstract class used for defining properties and methods used by different experiment setups.

    properties (Access = protected)
        % measurement used for every iteration
        measurements (:, 1) PlantMeasurement % (n_veh, 1)

        dt_seconds (1, 1) double
        amount (1, 1) double

        % used by labs
        Hp (1, 1) double
        manual_control_config ManualControlConfig % (1, 1)

    end

    properties (GetAccess = public, SetAccess = protected)
        % public so that the HLC can access them
        all_vehicle_indices (1, :) double
        vehicle_indices_controlled (1, :) double
    end

    methods (Static, Access = public)

        function plant = get_plant(environment, ros2_node)

            arguments
                environment (1, 1) Environment
                ros2_node = []
            end

            switch (environment)
                case Environment.Simulation
                    assert(~isempty(ros2_node))
                    plant = Simulation();
                    plant.set_ros2_node(ros2_node);
                case Environment.CpmLab
                    plant = CpmLab();
                case Environment.UnifiedLabApi
                    plant = UnifiedLabApi();
            end

        end

    end

    methods (Abstract)
        [cav_measurements, hdv_measurements] = measure(obj)
        apply(obj, info, experiment_result, k, mpa)
        got_stop = is_stop(obj)
        end_run(obj)
    end

    methods (Access = public)

        function obj = Plant()
        end

        function setup(obj, options)

            arguments
                obj (1, 1) Plant
                options (1, 1) Config
            end

            % This function does everything in order to run the object
            % later on. If further initialization needs to be done this
            % method shall be overwritten and called in a child class.

            % save options that are required as properties in subclasses
            obj.dt_seconds = options.dt_seconds;
            obj.amount = options.amount;
            obj.Hp = options.Hp;
            obj.manual_control_config = options.manual_control_config;

            obj.all_vehicle_indices = 1:options.amount;

            % construct measurements
            obj.measurements(options.amount, 1) = PlantMeasurement();
        end

        function register_map(~)
            % Plants may allow to set a specific map.
            % If so, this function needs to be overwritten.
            % By default this function is not available.
            error('This interface does not provide the possibility to register a specific map.');
        end

        function receive_map(~)
            % Plants may allow to retrieve a lab specific map.
            % If so, this function needs to be overwritten.
            % By default this function is not available.
            error('This interface does not provide the possibility to retrieve a lab specific map.');
        end

        function dt_seconds = get_step_time(obj)
            % return step time that is set by the plant
            dt_seconds = obj.dt_seconds;
        end

        function synchronize_start_with_plant(~)
            % can be overwritten in child class if needed
        end

    end

end
