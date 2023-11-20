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

            switch options.optimizer_type
                case OptimizerType.MatlabOptimal
                    optimizer = GraphSearch();
                case OptimizerType.MatlabSampled
                    optimizer = MonteCarloTreeSearch();
                case OptimizerType.CppOptimal

                    if options.is_prioritized
                        optimizer = GraphSearchMexPB(options, mpa, scenario, indices_in_vehicle_list, CppOptimizer.GraphSearchPBOptimal);
                    else
                        optimizer = GraphSearchMexCentralized(options, mpa, scenario, CppOptimizer.CentralizedOptimalPolymorphic);
                    end

                case OptimizerType.CppSampled

                    if options.is_prioritized
                        error('CppSampled can only be used in centralized execution yet.');
                    else
                        optimizer = GraphSearchMexCentralized(options, mpa, scenario, CppOptimizer.CentralizedNaiveMonteCarloPolymorphicParallel);
                    end

            end

        end

    end

end
