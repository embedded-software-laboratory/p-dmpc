classdef GraphSearchMexCentralized < OptimizerInterface

    properties (Access = protected)
        cpp_optimizer CppOptimizer; % Dependent on the optimizer_typein the config the corresponding cpp interface is selected
    end

    methods

        function obj = GraphSearchMexCentralized(options, mpa, scenario, cpp_optimizer)
            obj = obj@OptimizerInterface();

            obj.cpp_optimizer = cpp_optimizer;

            graph_search_cpp_centralized_mex(CppOptimizer.InitializeMex, options, mpa, scenario);
        end

        function info = run_optimizer(obj, ~, iter, ~, options)
            [ ...
                 next_nodes, ...
                 current_and_predicted_trims, ...
                 y_predicted_full_res ...
             ] = graph_search_cpp_centralized_mex(obj.cpp_optimizer, iter);

            info = ControlResultsInfo(iter.amount, options.Hp, iter.vehicle_ids);
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

end
