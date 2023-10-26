classdef PrioritizedOptimalController < PrioritizedController

    properties (Access = private)
        iter_base
        info_base
        iter_array_tmp (1, :) cell
        info_array_tmp (1, :) cell
    end

    methods

        function obj = PrioritizedOptimalController(scenario, plant)
            obj = obj@PrioritizedController(scenario, plant);
            obj.prioritizer = OptimalPrioritizer();
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

                assign_priority_timer = tic;
                obj.prioritize();
                obj.iter.timer.assign_priority = toc(assign_priority_timer);

                obj.weigh();

                obj.reduce();

                group_vehicles_timer = tic;
                obj.group();
                obj.iter.timer.group_vehs = toc(group_vehicles_timer);

                % plan for vehicle_idx
                runtime_planning = obj.plan_single_vehicle(vehicle_idx);

                %% Send own data to other vehicles
                msg_send_time = obj.publish_predictions(vehicle_idx);

                obj.info.computation_levels = length(obj.CL_based_hierarchy);
                % temporarily store dwata
                obj.iter_array_tmp{obj.iter.priority_permutation} = obj.iter;
                obj.info_array_tmp{obj.iter.priority_permutation} = obj.info;

            end

            [chosen_solution, ~, ~] = obj.choose_solution_cost();

            obj.info = obj.info_array_tmp{chosen_solution};
            obj.iter = obj.iter_array_tmp{chosen_solution};

            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = runtime_planning;
            obj.info.runtime_graph_search_max = obj.info.runtime_graph_search_each_veh(vehicle_idx);
            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = msg_send_time + runtime_planning;
            obj.info.runtime_subcontroller_each_veh(vehicle_idx) = obj.info.runtime_subcontroller_each_veh(vehicle_idx) + runtime_others;
            obj.info.runtime_subcontroller_max = obj.info.runtime_subcontroller_each_veh(vehicle_idx);
            obj.info.computation_levels = length(obj.CL_based_hierarchy);
            obj.iter.lanelet_crossing_areas = obj.lanelet_crossing_areas;
        end

    end

    methods (Access = protected)

        function runtime_others = init_step(obj)
            runtime_others_tic = tic;
            n_veh = obj.scenario.options.amount;
            Hp = obj.scenario.options.Hp;
            obj.info_base = ControlResultsInfo(n_veh, Hp, obj.plant.all_vehicle_ids);
            determine_couplings_timer = tic;
            obj.couple();
            obj.iter_base = obj.iter;
            obj.iter_base.timer.determine_couplings = toc(determine_couplings_timer);
            obj.iter_base.num_couplings_between_grps = 0; % number of couplings between groups
            obj.iter_base.num_couplings_between_grps_ignored = 0; % ignored number of couplings between groups by using lanelet crossing lanelets

            obj.iter_array_tmp = {};
            obj.info_array_tmp = {};
            runtime_others = toc(runtime_others_tic);
        end

    end

    methods (Access = private)

        function [chosen_solution, decision, fallback_solutions] = choose_solution_cost(obj)
            n_solutions = length(obj.iter_array_tmp);
            decision = zeros(n_solutions, 1);
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

                    decision(i_solution) = g_end;

                end

                % broadcast info about solution
                obj.decision_communication{obj.plant.indices_in_vehicle_list(1)}.send_message(obj.k, decision);

                % receive info about solutions
                other_vehicles = setdiff(1:obj.scenario.options.amount, obj.plant.indices_in_vehicle_list);

                for kVeh = other_vehicles
                    % loop over vehicle from which the messages are read
                    latest_msg_i = obj.decision_communication{obj.plant.indices_in_vehicle_list(1)}.read_message( ...
                        obj.plant.all_vehicle_ids(kVeh), ...
                        obj.k, ...
                        throw_error = true ...
                    );
                    % calculate objective value
                    decision_i = latest_msg_i.decision;
                    fallback_solutions(decision_i >= 1e6) = true;
                    decision = decision + decision_i;
                end

            end

            % disp(decision)

            % choose accordingly
            [~, chosen_solution] = min(decision);
            chosen_solution = chosen_solution(1); % guarantee that it is a single integer
        end

    end

end
