classdef (Abstract) OptimizerInterface < handle

    properties (Access = protected)
    end

    methods

        function obj = OptimizerInterface()
        end

    end

    methods (Abstract)
        info = run_optimizer(obj, veh_index, iter, mpa, options);
    end

    methods (Static)

        function optimizer = get_optimizer(options, mpa, scenario, indices_in_vehicle_list)
            %GET_OPTIMIZER creates an optimizer dependent on the select Matlab/CppOptimizer
            arguments
                options (1, 1) Config;
                mpa (1, 1) MotionPrimitiveAutomaton;
                scenario (1, 1) Scenario;
                indices_in_vehicle_list (1, :) double;
            end

            if (~xor(options.matlab_optimizer == MatlabOptimizer.none, options.cpp_optimizer == CppOptimizer.None))
                error('Exactly one optimizer needs to be selected! Current selection: ' ...
                    + string(options.matlab_optimizer) + ' (Matlab), ' ...
                    + string(options.cpp_optimizer) + ' (Cpp)');
            end

            switch (options.matlab_optimizer)
                case MatlabOptimizer.optimal
                    optimizer = GraphSearch();
                case MatlabOptimizer.randomized
                    error('The randomized Matlab optimizer is not yet implemented!');
            end

            if options.matlab_optimizer ~= MatlabOptimizer.none
                return;
            end

            switch (options.cpp_optimizer)
                case CppOptimizer.CentralizedOptimalPolymorphic
                    optimizer = GraphSearchMexCentralized(options, mpa, scenario);
                case CppOptimizer.GraphSearchPBOptimal
                    optimizer = GraphSearchMexPB(options, mpa, scenario, indices_in_vehicle_list);
                case CppOptimizer.CentralizedNaiveMonteCarloPolymorphicParallel
                    optimizer = GraphSearchMexCentralized(options, mpa, scenario);
                otherwise
                    error('The selected cpp_optimizer option is not available: ' + string(options.cpp_optimizer));
            end

        end

    end

end
