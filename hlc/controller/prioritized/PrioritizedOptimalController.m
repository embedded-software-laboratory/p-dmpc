classdef PrioritizedOptimalController < PrioritizedController

    properties (Access = private)
        iter_base
        info_base
        iter_array_tmp (1, :) cell
        info_array_tmp (1, :) cell
        permutations (1, :) cell
        n_trials = 300;
    end

    methods

        function obj = PrioritizedOptimalController(scenario, vehicle_ids)
            obj = obj@PrioritizedController(scenario, vehicle_ids);
            obj.n_trials = obj.scenario.options.trials_randomized_optimizer;
            obj.prioritizer = OptimalPrioritizer();
        end

    end

    methods (Access = protected)

        function precompute_n_solutions(obj, n, iter)
            % generate <n> random paths of length Hp

            for i = 1:n
                obj.optimizer.precompute_solution(iter);
            end

        end

        function controller(obj)
            % PB_CONTROLLER_PARL Plan trajectory for one time step using a
            % priority-based controller. Vehicles inside one group plan in sequence and
            % between groups plan in pararllel. Controller simulates multiple
            % distributed controllers in a for-loop.
            % TODO: change description according to explorative controller
            obj.init_step();

            vehicle_idx = obj.indices_in_vehicle_list(1);

            for permutation = 1:factorial(obj.scenario.options.amount)

                obj.iter.permutation = permutation;

                obj.iter = obj.iter_base;

                obj.prioritize();
                obj.weigh();
                obj.reduce();
                obj.group();
                obj.compute_belonging_vector();

                % plan for vehicle_idx
                runtime_planning = obj.plan_single_vehicle(vehicle_idx);

                %% Send own data to other vehicles
                obj.publish_predictions(vehicle_idx);

                obj.info.runtime_subcontroller_each_veh(vehicle_idx) = runtime_planning;

                obj.info.computation_levels = length(obj.CL_based_hierarchy);
                % temporarily store dwata
                obj.iter_array_tmp{obj.iter.permutation} = obj.iter;
                obj.info_array_tmp{obj.iter.permutation} = obj.info;
            end

            [chosen_solution, ~, ~] = obj.choose_solution_cost();

            obj.info = obj.info_array_tmp{chosen_solution};
            obj.iter = obj.iter_array_tmp{chosen_solution};

        end

    end

    methods (Access = protected)

        function init_step(obj)
            n_veh = obj.scenario.options.amount;
            Hp = obj.scenario.options.Hp;
            obj.info_base = ControlResultsInfo(n_veh, Hp, [obj.scenario.vehicles.ID]);
            obj.couple();

            obj.iter_base = obj.iter;
            obj.iter_array_tmp = {};
            obj.info_array_tmp = {};
        end

    end

    methods (Access = private)

        function [chosen_solution, decision, fallback_solutions] = choose_solution_cost(obj)
            n_solutions = length(obj.iter_array_tmp);
            decision = zeros(n_solutions, 1);
            fallback_solutions = false(n_solutions, 1);

            if n_solutions > 1

                for i_solution = 1:n_solutions

                    if ismember(obj.indices_in_vehicle_list(1), obj.info_array_tmp{i_solution}.vehs_fallback)
                        fallback_solutions(i_solution) = true;
                        % in case of fallback use maximum cost
                        g_end = 1e9;
                    else
                        % prefer solutions that are computed and have a low cost to come value in the last step
                        g_end = obj.info_array_tmp{i_solution}.tree{obj.indices_in_vehicle_list(1)}.g(obj.info_array_tmp{i_solution}.tree_path(obj.indices_in_vehicle_list(1), end));
                    end

                    decision(i_solution) = g_end;

                end

                % broadcast info about solution
                obj.scenario.vehicles(obj.indices_in_vehicle_list(1)).communicate.decision.send_message(obj.k, decision);

                % receive info about solutions
                other_vehicles = setdiff(1:obj.scenario.options.amount, obj.indices_in_vehicle_list);
                read_tic = tic;
                latest_msgs = read_messages(obj.scenario.vehicles(obj.indices_in_vehicle_list(1)).communicate.decision, obj.k, obj.scenario.options.amount - 1);
                obj.read_time = obj.read_time + toc(read_tic);
                % calculate objective value
                for i_veh = other_vehicles
                    latest_msg_i = latest_msgs(find([latest_msgs.vehicle_id] == obj.scenario.options.veh_ids(i_veh), 1));
                    decision_i = latest_msg_i.decision;
                    fallback_solutions(decision_i >= 1e6) = true;
                    decision = decision + decision_i;
                end

            end

            % choose accordingly
            [~, chosen_solution] = min(decision);
            chosen_solution = chosen_solution(1); % guarantee that it is a single integer

            obj.info = obj.info_array_tmp{chosen_solution};
            obj.iter = obj.iter_array_tmp{chosen_solution};
        end

        function swap_entries_all_coupling_matrices(obj, veh_i, veh_j)
            entry_1 = (veh_i - 1) * obj.scenario.options.amount + veh_j;
            entry_2 = (veh_j - 1) * obj.scenario.options.amount + veh_i;
            entries = [entry_1, entry_2];
            swapped_entries = fliplr(entries);
            obj.iter.weighted_coupling(entries) = obj.iter_base.weighted_coupling(swapped_entries);
            obj.iter.weighted_coupling_reduced(entries) = obj.iter_base.weighted_coupling_reduced(swapped_entries);
            obj.iter.directed_coupling(entries) = obj.iter_base.directed_coupling(swapped_entries);
            obj.iter.directed_coupling_reduced(entries) = obj.iter_base.directed_coupling_reduced(swapped_entries);
        end

    end

end
