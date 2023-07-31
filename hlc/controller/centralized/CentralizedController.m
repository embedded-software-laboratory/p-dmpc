classdef CentralizedController < HighLevelController

    methods

        function obj = CentralizedController(scenario, plant)
            obj = obj@HighLevelController(scenario, plant);

            if obj.scenario.options.use_cpp
                obj.optimizer = GraphSearchMexCentralized(obj.scenario, obj.plant.indices_in_vehicle_list);
            else
                obj.optimizer = GraphSearch(obj.scenario);
            end

        end

    end

    methods (Access = protected)

        function controller(obj)
            % initialize variable to store control results
            obj.info = ControlResultsInfo(obj.scenario.options.amount, obj.scenario.options.Hp, obj.plant.all_veh_ids);

            % falsifies controller_runtime slightly
            subcontroller_timer = tic;

            [info_v, ~] = obj.optimizer.run_optimizer(obj.iter, obj.plant.indices_in_vehicle_list);

            if info_v.is_exhausted
                info_v = handle_graph_search_exhaustion(info_v, obj.scenario, obj.iter);
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
                obj.info = store_control_info(obj.info, info_v, obj.scenario);
            end

            obj.info.runtime_subcontroller_each_veh = toc(subcontroller_timer);
            obj.info.runtime_graph_search_each_veh = obj.info.runtime_subcontroller_each_veh;
            % for centralize controller, all vehicles are in the same group
            obj.info.runtime_subcontroller_each_grp = obj.info.runtime_subcontroller_each_veh;
            obj.info.runtime_subcontroller_max = obj.info.runtime_subcontroller_each_veh;
            obj.info.runtime_graph_search_max = obj.info.runtime_subcontroller_each_veh;
        end

    end

end
