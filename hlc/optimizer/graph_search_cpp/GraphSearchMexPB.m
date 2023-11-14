classdef GraphSearchMexPB < OptimizerInterface

    properties (Access = protected)
        mexhosts = matlab.mex.MexHost.empty;
    end

    methods

        function obj = GraphSearchMexPB(options, mpa, scenario, vehicle_indices)
            obj = obj@OptimizerInterface();
            % When using C++, you don't want to send the scenario over
            % and over again, so it is done in the init function

            if ~options.mex_out_of_process_execution
                % if mex is not executed out of the Matlab process
                graph_search_cpp_priority_mex(CppOptimizer.InitializeMex, options, mpa, scenario);
                return
            end

            for vehicle_index = vehicle_indices
                % create mexhost for each vehicle (only if incremental search is used - no option in config yet)
                obj.mexhosts(vehicle_index) = mexhost;
                % initialize C++ graph search
                feval(obj.mexhosts(vehicle_index), 'graph_search_cpp_priority_mex', CppOptimizer.InitializeMex, options, mpa, scenario);
            end

        end

        function info_v = run_optimizer(obj, veh_index, iter, ~, options)
            info_v = ControlResultsInfo(iter.amount, options.Hp, iter.vehicle_ids);

            [vehicle_obstacles, ~] = get_all_obstacles(iter, options.Hp);

            [next_nodes, info_v.predicted_trims, info_v.y_predicted, info_v.shapes, info_v.n_expanded, info_v.is_exhausted] = obj.do_graph_search(veh_index, iter, vehicle_obstacles, options);
            info_v.tree = obj.create_tree(iter);
            info_v.tree_path = 1:(options.Hp + 1);

            for i = 1:options.Hp
                info_v.tree.add_node(i, next_nodes{i});
            end

        end

    end

    methods (Access = private)

        function tree = create_tree(~, iter)
            trim = iter.trim_indices;
            x = iter.x0(:, 1);
            y = iter.x0(:, 2);
            yaw = iter.x0(:, 3);
            k = 0;
            g = 0;
            h = 0;
            tree = AStarTree(x, y, yaw, trim, k, g, h);
        end

        function [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = do_graph_search(obj, veh_index, iter, vehicle_obstacles, options)

            if options.mex_out_of_process_execution
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    feval(obj.mexhosts(veh_index(1)), 'graph_search_cpp_priority_mex', options.cpp_optimizer, iter, vehicle_obstacles);
            else
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    graph_search_cpp_priority_mex(options.cpp_optimizer, iter, vehicle_obstacles);
            end

        end

    end

end
