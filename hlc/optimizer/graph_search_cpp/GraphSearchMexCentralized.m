classdef GraphSearchMexCentralized < OptimizerInterface

    methods

        function obj = GraphSearchMexCentralized(scenario)
            obj = obj@OptimizerInterface(scenario);

            graph_search_cpp_centralized_mex(Function.InitializeWithScenario, obj.scenario);
        end

    end

    methods

        function [info_v, graph_search_time] = run_optimizer(obj, iter, veh_index)
            graph_search_timer = tic;
            graph_search_data = GraphSearchData(iter, obj.scenario, veh_index);
            info_v = ControlResultsInfo(iter.amount, obj.scenario.options.Hp, [iter.vehicles.ID]);
            
            [next_nodes, info_v.predicted_trims, info_v.y_predicted] = graph_search_cpp_centralized_mex(obj.scenario.options.cpp_implementation, iter);
            
            graph_search_time = toc(graph_search_timer);
            info_v.tree = obj.create_tree(iter);
            info_v.tree_path = 1:(obj.scenario.options.Hp + 1);

            for i = 1:obj.scenario.options.Hp
                info_v.tree.add_node(i, next_nodes{i});
            end

        end

    end

    %methods (Abstract, Access = protected)
    %    [next_nodes, predicted_trims, y_predicted] = do_graph_search(iter)
    %end

    methods (Access = protected)

        function tree = create_tree(obj, iter)
            trim = iter.trim_indices;
            x = iter.x0(:, 1);
            y = iter.x0(:, 2);
            yaw = iter.x0(:, 3);
            k = 0;
            g = 0;
            h = 0;
            tree = Tree(x, y, yaw, trim, k, g, h);
        end
    end

    %methods (Access = private)
%
    %    function [next_nodes, predicted_trims, y_predicted, n_expanded, is_exhausted] = do_graph_search(obj, graph_search_data)
%
    %        if obj.scenario.options.mex_out_of_process_execution
    %            [next_nodes, predicted_trims, y_predicted, n_expanded, is_exhausted] = ...
    %                feval(obj.mexhosts(graph_search_data.veh_idx(1)), 'optimizer', Function.GraphSearchCentralizedOptimal, graph_search_data);
    %        else
    %            [next_nodes, predicted_trims, y_predicted, n_expanded, is_exhausted] = ...
    %                optimizer(Function.GraphSearchCentralizedOptimal, graph_search_data);
    %        end
%
    %    end
%
    %end

end
