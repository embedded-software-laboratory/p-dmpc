classdef CentralizedController < HighLevelController

    properties
    end

    methods

        function obj = CentralizedController(options, scenario, plant)
            obj = obj@HighLevelController(options, scenario, plant);
        end

    end

    methods (Access = protected)

        function init(obj)
            % initialize superclass
            init@HighLevelController(obj);

            % construct optimizer
            if obj.options.use_cpp()
                obj.optimizer = GraphSearchMexCentralized(obj.scenario, obj.mpa);
            else
                obj.optimizer = GraphSearch(obj.scenario, obj.mpa);
            end

        end

        function create_coupling_graph(obj)

            obj.timing_general.start('coupling', obj.k);

            if obj.options.use_cpp()
                obj.iter.adjacency = obj.coupler.couple(obj.iter);
            end

            obj.timing_general.stop('coupling', obj.k);

        end

        function controller(obj)
            % initialize variable to store control results
            obj.info = ControlResultsInfo(obj.scenario.options.amount, obj.scenario.options.Hp, obj.plant.all_vehicle_ids);

            obj.timing_general.start('optimizer', obj.k);
            info_v = obj.optimizer.run_optimizer(obj.iter, obj.plant.indices_in_vehicle_list);
            obj.timing_general.stop('optimizer', obj.k);

            obj.timing_general.start('fallback', obj.k);

            if info_v.is_exhausted
                info_v = handle_graph_search_exhaustion(info_v, obj.options, obj.iter, obj.mpa);
            end

            if info_v.needs_fallback
                % if graph search is exhausted, this vehicles and all vehicles that have directed or
                % undirected couplings with this vehicle will take fallback
                disp(['Graph search exhausted at time step: ' num2str(obj.k) '.'])
                % all vehicles fall back
                obj.info.vehs_fallback = 1:obj.scenario.options.amount;
                obj.info.needs_fallback(obj.info.vehs_fallback) = true;
            else
                % prepare output data
                obj.info = store_control_info(obj.info, info_v, obj.options, obj.mpa);
            end

            obj.timing_general.stop('fallback', obj.k);

        end

        function plan_for_fallback(~)
            % TODO must be implemented! (see issue #142)
            error(['No fallback handling for centralized controller', ...
                   ' implemented yet!'])
        end

    end

end
