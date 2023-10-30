classdef PrioritizedParallelOptimalController < PrioritizedController

    properties (Access = private)
        iter_base
        info_base
        iter_array_tmp (1, :) cell
        info_array_tmp (1, :) cell
    end

    methods

        function obj = PrioritizedParallelOptimalController(scenario, plant)
            obj = obj@PrioritizedController(scenario, plant);
            obj.prioritizer = ConstantPrioritizer();
        end

    end

    methods (Access = protected)

        function controller(obj)
            runtime_others = obj.init_step();

            vehicle_idx = obj.plant.indices_in_vehicle_list(1);

            for priority_permutation = 1:factorial(obj.scenario.options.amount)

                obj.iter = obj.iter_base;
                obj.info = obj.info_base;
                obj.iter.priority_permutation = priority_permutation;

                obj.prioritize();
                obj.reduce_computation_levels();

                % plan for vehicle_idx
                planning_timer = tic;
                obj.plan_single_vehicle(vehicle_idx);
                runtime_planning = toc(planning_timer);

                %% Send own data to other vehicles
                msg_send_tic = tic;
                obj.publish_predictions(vehicle_idx);
                msg_send_time = toc(msg_send_tic);

                obj.info.computation_levels = length(obj.CL_based_hierarchy);
                % temporarily store data
                obj.iter_array_tmp{obj.iter.priority_permutation} = obj.iter;
                obj.info_array_tmp{obj.iter.priority_permutation} = obj.info;

            end

            [chosen_solution, ~, ~] = obj.choose_solution_cost();

            obj.info = obj.info_array_tmp{chosen_solution};
            obj.iter = obj.iter_array_tmp{chosen_solution};

            obj.info.runtime_graph_search_max = obj.info.runtime_graph_search_each_veh(vehicle_idx);
            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = msg_send_time + runtime_planning;
            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = obj.info.runtime_subcontroller_each_veh(vehicle_idx);
            obj.info.runtime_subcontroller_max = obj.info.runtime_subcontroller_each_veh(vehicle_idx);
            obj.info.computation_levels = length(obj.CL_based_hierarchy);
        end

    end

    methods (Access = protected)

        function runtime_others = init_step(obj)
            runtime_others_tic = tic;
            n_veh = obj.scenario.options.amount;
            Hp = obj.scenario.options.Hp;
            obj.info_base = ControlResultsInfo(n_veh, Hp, obj.plant.all_vehicle_ids);
            obj.couple();
            obj.iter_base = obj.iter;

            obj.iter_array_tmp = {};
            obj.info_array_tmp = {};
            runtime_others = toc(runtime_others_tic);
        end

    end

    methods (Access = private)

        function [chosen_solution, solution_cost, fallback_solutions] = choose_solution_cost(obj)
            n_solutions = length(obj.iter_array_tmp);
            solution_cost = zeros(n_solutions, 1);
            fallback_solutions = false(n_solutions, 1);

            if n_solutions > 1

                for i_solution = 1:n_solutions

                    if ismember(obj.plant.indices_in_vehicle_list(1), obj.info_array_tmp{i_solution}.vehs_fallback)
                        fallback_solutions(i_solution) = true;
                        % in case of fallback use maximum cost
                        g_end = 1e9;
                    else
                        % prefer solutions that are computed and have a low cost to come value in the last step
                        g_end = obj.info_array_tmp{i_solution}.tree{obj.plant.indices_in_vehicle_list(1)}.g(obj.info_array_tmp{i_solution}.tree_path(obj.plant.indices_in_vehicle_list(1), end));
                    end

                    solution_cost(i_solution) = g_end;

                end

                % broadcast info about solution
                obj.solution_cost_communication{obj.plant.indices_in_vehicle_list(1)}.send_message(obj.k, solution_cost);

                % receive info about solutions
                other_vehicles = setdiff(1:obj.scenario.options.amount, obj.plant.indices_in_vehicle_list);

                for kVeh = other_vehicles
                    % loop over vehicle from which the messages are read
                    latest_msg_i = obj.solution_cost_communication{obj.plant.indices_in_vehicle_list(1)}.read_message( ...
                        obj.plant.all_vehicle_ids(kVeh), ...
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
