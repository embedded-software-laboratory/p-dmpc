classdef CentralizedController < HLCInterface
    methods
        function obj = CentralizedController()
            obj = obj@HLCInterface();
        end
    end
    methods (Access = protected)
        function controller(obj)
            % initialize variable to store control results
            obj.info = ControllResultsInfo(obj.scenario.options.amount, obj.scenario.options.Hp, [obj.scenario.vehicles.ID]);

            % falsifies controller_runtime slightly
            subcontroller_timer = tic;

            if obj.scenario.options.use_cpp
                [info_v.next_node, info_v.predicted_trims, info_v.y_predicted] = ...
                optimizer(Function.GraphSearchCentralizedOptimalPolymorphic, obj.iter);

                %optimizer(Function.GraphSearchCentralizedOptimalPolymorphicSpeedHeuristic, obj.iter);
                %optimizer(Function.GraphSearchCentralizedOptimalPolymorphic, obj.iter);
                %optimizer(Function.GraphSearchCentralizedOptimalMemorySaving, obj.iter);
                %optimizer(Function.GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic, obj.iter);
                %optimizer(Function.GraphSearchCentralizedParallelNaiveMonteCarlo, obj.iter);
                %optimizer(Function.GraphSearchCentralizedOptimal, obj.iter);
                %optimizer(Function.GraphSearchCentralizedNaiveMonteCarloPolymorphic, obj.iter);
                %optimizer(Function.GraphSearchCentralizedNaiveMonteCarlo, obj.iter);
                
                

                obj.info = store_control_info(obj.info, info_v, obj.scenario);
            else
                info_v = obj.sub_controller(obj.scenario, obj.iter);
                if info_v.is_exhausted
                    % if graph search is exhausted, this vehicles and all vehicles that have directed or
                    % undirected couplings with this vehicle will take fallback
                    disp(['Graph search exhausted at time step: ' num2str(obj.k) '.'])
                    % all vehicles fall back
                    obj.info.vehs_fallback = 1:obj.scenario.options.amount;
                    obj.info.is_exhausted(obj.info.vehs_fallback) = true;
                else
                    % prepare output data
                   obj.info = store_control_info(obj.info, info_v, obj.scenario);
                end
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
