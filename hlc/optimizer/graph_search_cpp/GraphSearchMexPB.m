classdef GraphSearchMexPB < OptimizerInterface

    properties (Access = protected)
        mexhosts = matlab.mex.MexHost.empty;
        cpp_optimizer CppOptimizer; % Dependent on the optimizer_type in the config the corresponding cpp interface is selected
    end

    methods

        function obj = GraphSearchMexPB(options, mpa, scenario, vehicle_indices, cpp_optimizer)
            obj = obj@OptimizerInterface();
            % When using C++, you don't want to send the scenario over
            % and over again, so it is done in the init function

            obj.cpp_optimizer = cpp_optimizer;

            if ~options.mex_out_of_process_execution
                % if mex is not executed out of the Matlab process
                graph_search_cpp_priority_mex(CppOptimizer.InitializeMex, options, mpa, scenario);
                return
            end

            for i_vehicle = vehicle_indices
                % create mexhost for each vehicle (only if incremental search is used - no option in config yet)
                obj.mexhosts(i_vehicle) = mexhost;
                % initialize C++ graph search
                feval(obj.mexhosts(i_vehicle), 'graph_search_cpp_priority_mex', CppOptimizer.InitializeMex, options, mpa, scenario);
            end

        end

        function info = run_optimizer(obj, vehicle_index, iter, ~, options, ~)
            info = ControlResultsInfo(iter.amount, options.Hp);

            [vehicle_obstacles, ~] = get_all_obstacles(iter, options.Hp);

            [ ...
                 next_nodes, ...
                 current_and_predicted_trims, ...
                 y_predicted_full_res, ...
                 info.shapes, ...
                 info.n_expanded, ...
                 info.is_exhausted ...
             ] = obj.do_graph_search(vehicle_index, iter, vehicle_obstacles, options);

            info = OptimizerInterface.create_control_results_info_from_mex( ...
                info, ...
                iter, ...
                options, ...
                next_nodes, ...
                current_and_predicted_trims, ...
                y_predicted_full_res ...
            );

        end

    end

    methods (Access = private)

        function [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = do_graph_search(obj, veh_index, iter, vehicle_obstacles, options)

            if options.mex_out_of_process_execution
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    feval(obj.mexhosts(veh_index(1)), 'graph_search_cpp_priority_mex', obj.cpp_optimizer, iter, vehicle_obstacles);
            else
                [next_nodes, predicted_trims, y_predicted, shapes, n_expanded, is_exhausted] = ...
                    graph_search_cpp_priority_mex(obj.cpp_optimizer, iter, vehicle_obstacles);
            end

        end

    end

end
