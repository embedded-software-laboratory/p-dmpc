classdef CentralizedController < HighLevelController

    properties
    end

    methods

        function obj = CentralizedController(options, plant)
            obj = obj@HighLevelController(options, plant);
        end

    end

    methods (Access = protected)

        function init(obj)
            % initialize superclass
            init@HighLevelController(obj);

            % construct optimizer
            obj.optimizer = OptimizerInterface.get_optimizer(obj.options, obj.mpa, obj.scenario_adapter.scenario, obj.plant.vehicle_indices_controlled);

        end

        function create_coupling_graph(obj)

            obj.timing.start('coupling', obj.k);

            if obj.options.use_cpp()
                obj.iter.adjacency = obj.coupler.couple(obj.options, obj.mpa.get_max_speed_of_mpa(), obj.scenario_adapter.scenario.adjacency_lanelets, obj.iter);
            end

            obj.timing.stop('coupling', obj.k);

        end

        function controller(obj)
            % initialize variable to store control results
            obj.info = ControlResultsInfo(obj.options.amount, obj.options.Hp);

            obj.timing.start('optimizer', obj.k);
            obj.info = obj.optimizer.run_optimizer( ...
                obj.plant.vehicle_indices_controlled, ...
                obj.iter, ...
                obj.mpa, ...
                obj.options ...
            );
            obj.timing.stop('optimizer', obj.k);

            obj.timing.start('fallback', obj.k);

            obj.info.needs_fallback = obj.info.is_exhausted;

            if obj.info.needs_fallback
                % if graph search is exhausted, this vehicles and all vehicles that have directed or
                % undirected couplings with this vehicle will take fallback
                disp(['Graph search exhausted at time step: ' num2str(obj.k) '.'])
                % all vehicles fall back
                obj.info.vehicles_fallback = 1:obj.options.amount;
                obj.info.needs_fallback(obj.info.vehicles_fallback) = true;
            end

            obj.timing.stop('fallback', obj.k);

        end

        function plan_for_fallback(~)
            error(['No fallback handling for centralized controller', ...
                   ' implemented yet!'])
        end

    end

end
