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

        function optimizer = get_optimizer(options, mpa, scenario, vehicle_indices_controlled)
            %GET_OPTIMIZER creates an optimizer dependent on the select Matlab/CppOptimizer
            arguments
                options (1, 1) Config;
                mpa (1, 1) MotionPrimitiveAutomaton;
                scenario (1, 1) Scenario;
                vehicle_indices_controlled (1, :) double;
            end

            switch options.optimizer_type
                case OptimizerType.MatlabOptimal
                    optimizer = GraphSearch();
                    OptimizerInterface.set_constraint_checker(optimizer, options);
                case OptimizerType.MatlabSampled
                    optimizer = MonteCarloTreeSearch();
                    OptimizerInterface.set_constraint_checker(optimizer, options);

                case OptimizerType.CppOptimal

                    if options.is_prioritized
                        optimizer = GraphSearchMexPB(options, mpa, scenario, vehicle_indices_controlled, CppOptimizer.GraphSearchPBOptimal);
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

        function set_constraint_checker(optimizer, options)

            if options.are_any_obstacles_non_convex
                optimizer.set_up_constraints = @vectorize_all_obstacles;
                optimizer.are_constraints_satisfied = @are_constraints_satisfied_interx;
            else
                optimizer.set_up_constraints = @(~, ~)deal([]);
                optimizer.are_constraints_satisfied = @are_constraints_satisfied_sat;
            end

        end

    end

    methods (Static, Access = protected)

        function tree = create_tree(iter)
            trim = iter.trim_indices;
            x = iter.x0(:, 1);
            y = iter.x0(:, 2);
            yaw = iter.x0(:, 3);
            k = 0;
            g = 0;
            h = 0;
            tree = Tree(x, y, yaw, trim, k, g, h);
        end

        function info = create_control_results_info_from_mex( ...
                info, ...
                iter, ...
                options, ...
                next_nodes, ...
                current_and_predicted_trims, ...
                y_predicted_full_res ...
            )

            info.predicted_trims = current_and_predicted_trims(:, 2:end);
            info.tree = OptimizerInterface.create_tree(iter);
            info.tree_path = 1:(options.Hp + 1);

            for i = 1:options.Hp
                info.tree.add_node_object(i, next_nodes{i});
            end

            % y_predicted_full_res is a cell array of size (options.amount x 1)
            % Each cell contains a matrix of size (n_predicted_points x 4)
            info.y_predicted = nan(3, options.Hp, iter.amount);

            if ~info.is_exhausted

                for i_vehicle = 1:iter.amount

                    n_predicted_points = size(y_predicted_full_res{i_vehicle}, 1);
                    entries_per_time_step = n_predicted_points / options.Hp;
                    idx_predicted_points = entries_per_time_step:entries_per_time_step:n_predicted_points;

                    for i_step = 1:options.Hp
                        i_predicted_point = idx_predicted_points(i_step);
                        info.y_predicted(:, i_step, i_vehicle) = y_predicted_full_res{i_vehicle}(i_predicted_point, 1:3);
                    end

                end

            end

        end

    end

end
