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
            obj.optimizer = OptimizerInterface.get_optimizer(obj.options);

        end

        function create_coupling_graph(obj)

            obj.timing.start('coupling', obj.k)

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
                obj.options, ...
                obj.k ...
            );
            obj.timing.stop('optimizer', obj.k);

            obj.timing.start('fallback', obj.k);

            obj.info.needs_fallback = obj.info.is_exhausted;

            if obj.info.needs_fallback
                % if graph search is exhausted, this vehicles and all vehicles that have directed or
                % undirected couplings with this vehicle will take fallback
                disp(['Graph search exhausted at time step: ' num2str(obj.k) '.'])
            end

            obj.timing.stop('fallback', obj.k);

        end

        function controller_fallback(~, ~)

            arguments
                ~
                ~
            end

            error(['No fallback handling for centralized controller', ...
                   ' implemented yet!'])
        end

    end

end
