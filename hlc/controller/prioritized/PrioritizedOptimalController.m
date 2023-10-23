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
            obj.init_step();

            vehicle_idx = obj.indices_in_vehicle_list(1);

            for permutation = 1:factorial(obj.scenario.options.amount)

                obj.iter = obj.iter_base;
                obj.info = obj.info_base;
                obj.iter.permutation = permutation;

                obj.prioritize();
                obj.weigh();
                obj.reduce();
                obj.group();

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

    end

end
