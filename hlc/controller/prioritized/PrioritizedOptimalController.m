classdef PrioritizedOptimalController < PrioritizedController

    properties (Access = protected)
        iter_base
        info_base
        iter_array_tmp (1, :) cell
        info_array_tmp (1, :) cell
        solution_cost (:, 1) double
    end

    methods

        function obj = PrioritizedOptimalController(options, plant, ros2_node)
            obj = obj@PrioritizedController(options, plant, ros2_node);
        end

    end

    methods (Access = protected)

        function controller(obj)
            obj.prepare_iterations();

            unique_priorities = Prioritizer.unique_priorities(obj.iter.adjacency);
            n_priorities = size(unique_priorities, 2);

            for priority_permutation = 1:n_priorities

                obj.prepare_permutation( ...
                    unique_priorities(:, priority_permutation), ...
                    priority_permutation ...
                );
                obj.solve_permutation();

            end

            obj.compute_solution_cost();
            obj.send_solution_cost();
            obj.receive_solution_cost();
            obj.choose_solution();

        end

        function create_coupling_graph(obj)
            obj.timing.start('receive_from_others', obj.k);
            obj.update_other_vehicles_traffic_info();
            obj.timing.stop('receive_from_others', obj.k);
            obj.timing.start('couple', obj.k);
            obj.couple();
            obj.timing.stop('couple', obj.k);
        end

    end

    methods (Access = public)

        function prepare_iterations(obj)
            % initialize variable to store control results
            obj.info_base = ControlResultsInfo( ...
                1, ...
                obj.options.Hp ...
            );

            % set base iteration data
            obj.iter_base = obj.iter;

            % initialize
            obj.iter_array_tmp = {};
            obj.info_array_tmp = {};
        end

        function prepare_permutation(obj, current_priorities, priority_permutation)

            obj.iter = obj.iter_base;
            obj.info = obj.info_base;
            obj.iter.priority_permutation = priority_permutation;

            obj.prioritizer.current_priorities = current_priorities;

            obj.prioritize();
            obj.group();
        end

        function solve_permutation(obj)

            obj.plan();

            % Send own data to other vehicles
            obj.publish_predictions();

            % temporarily store data
            obj.iter_array_tmp{obj.iter.priority_permutation} = obj.iter;
            obj.info_array_tmp{obj.iter.priority_permutation} = obj.info;
        end

        function compute_solution_cost(obj)

            n_solutions = length(obj.iter_array_tmp);
            obj.solution_cost = NaN(1, n_solutions);

            for i_solution = 1:n_solutions

                if obj.info_array_tmp{i_solution}.needs_fallback
                    % in case of fallback use maximum cost
                    cost_value = 1e9;
                else
                    % prefer solutions that are computed and have a low cost to come value in the last step
                    cost_value = obj.info_array_tmp{i_solution}.tree.get_cost( ...
                        obj.info_array_tmp{i_solution}.tree_path(end) ...
                    );
                end

                obj.solution_cost(i_solution) = cost_value;

            end

        end

        function send_solution_cost(obj)

            % broadcast info about solution
            obj.solution_cost_communication.send_message( ...
                obj.k, ...
                obj.solution_cost ...
            );
        end

        function receive_solution_cost(obj)
            % receive info about solutions
            other_vehicles = setdiff( ...
                1:obj.options.amount, ...
                obj.plant.vehicle_indices_controlled ...
            );

            for j_vehicle = other_vehicles
                % loop over vehicle from which the messages are read
                latest_msg_j = obj.solution_cost_communication.read_message( ...
                    j_vehicle, ...
                    obj.k, ...
                    throw_error = true ...
                );
                % calculate objective value
                obj.solution_cost = obj.solution_cost + latest_msg_j.solution_cost;
            end

        end

        function choose_solution(obj)
            [~, chosen_solution] = min(obj.solution_cost);
            chosen_solution = chosen_solution(1); % guarantee that it is a single integer

            obj.info = obj.info_array_tmp{chosen_solution};
            obj.iter = obj.iter_array_tmp{chosen_solution};
        end

    end

end
