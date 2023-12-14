classdef PrioritizedParallelOptimalController < PrioritizedController

    properties (Access = private)
        iter_base
        info_base
        iter_array_tmp (1, :) cell
        info_array_tmp (1, :) cell
    end

    methods

        function obj = PrioritizedParallelOptimalController(options, plant)
            obj = obj@PrioritizedController(options, plant);
            obj.prioritizer = ConstantPrioritizer();
        end

    end

    methods (Access = protected)

        function controller(obj)
            % initialize variable to store control results
            obj.info_base = ControlResultsInfo( ...
                obj.options.amount, ...
                obj.options.Hp, ...
                obj.plant.all_vehicle_indices ...
            );

            % set base iteration data
            obj.iter_base = obj.iter;

            % initialize
            obj.iter_array_tmp = {};
            obj.info_array_tmp = {};
            unique_priorities = Prioritizer.unique_priorities(obj.iter.adjacency);
            n_priorities = size(unique_priorities, 2);

            i_vehicle = obj.plant.vehicle_indices_controlled;

            for priority_permutation = 1:n_priorities

                obj.iter = obj.iter_base;
                obj.info = obj.info_base;
                obj.iter.priority_permutation = priority_permutation;

                obj.prioritizer.current_priorities = unique_priorities(:, priority_permutation);

                obj.prioritize();
                obj.reduce_computation_levels();

                % plan for i_vehicle
                obj.plan(i_vehicle);

                %% Send own data to other vehicles
                obj.publish_predictions(i_vehicle);

                % temporarily store data
                obj.iter_array_tmp{obj.iter.priority_permutation} = obj.iter;
                obj.info_array_tmp{obj.iter.priority_permutation} = obj.info;
            end

            [chosen_solution, ~, ~] = obj.choose_solution_cost();

            obj.info = obj.info_array_tmp{chosen_solution};
            obj.iter = obj.iter_array_tmp{chosen_solution};
        end

    end

    methods (Access = protected)

        function create_coupling_graph(obj)
            obj.update_other_vehicles_traffic_info();
            obj.timing.start('couple', obj.k);
            obj.couple();
            obj.timing.stop('couple', obj.k);
        end

    end

    methods (Access = private)

        function [chosen_solution, solution_cost, fallback_solutions] = choose_solution_cost(obj)
            n_solutions = length(obj.iter_array_tmp);
            solution_cost = zeros(n_solutions, 1);
            fallback_solutions = false(n_solutions, 1);

            if n_solutions > 1

                for i_solution = 1:n_solutions

                    if ismember(obj.plant.vehicle_indices_controlled(1), obj.info_array_tmp{i_solution}.vehicles_fallback)
                        fallback_solutions(i_solution) = true;
                        % in case of fallback use maximum cost
                        g_end = 1e9;
                    else
                        % prefer solutions that are computed and have a low cost to come value in the last step
                        g_end = obj.info_array_tmp{i_solution}.tree{obj.plant.vehicle_indices_controlled(1)}.g(obj.info_array_tmp{i_solution}.tree_path(obj.plant.vehicle_indices_controlled(1), end));
                    end

                    solution_cost(i_solution) = g_end;

                end

                % broadcast info about solution
                obj.solution_cost_communication{obj.plant.vehicle_indices_controlled(1)}.send_message(obj.k, solution_cost);

                % receive info about solutions
                other_vehicles = setdiff(1:obj.options.amount, obj.plant.vehicle_indices_controlled);

                for kVeh = other_vehicles
                    % loop over vehicle from which the messages are read
                    latest_msg_i = obj.solution_cost_communication{obj.plant.vehicle_indices_controlled(1)}.read_message( ...
                        kVeh, ...
                        obj.k, ...
                        throw_error = true ...
                    );
                    % calculate objective value
                    solution_cost_i = latest_msg_i.solution_cost;
                    fallback_solutions(solution_cost_i >= 1e6) = true;
                    solution_cost = solution_cost + solution_cost_i;
                end

            end

            % disp(solution_cost)

            % choose accordingly
            [~, chosen_solution] = min(solution_cost);
            chosen_solution = chosen_solution(1); % guarantee that it is a single integer
        end

    end

end
