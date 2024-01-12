classdef PrioritizedSequentialController < HighLevelController

    properties (Access = protected)
        hlcs (1, :) % PrioritizedController
    end

    methods

        function obj = PrioritizedSequentialController()
        end

        function add_hlc(obj, hlc)
            if isempty(obj.hlcs)
                % Set correct data type of hlcs
                if isa(hlc,'PrioritizedOptimalController')
                    % test for PrioritizedOptimalController first, since
                    % the child is also a parent
                    obj.hlcs = PrioritizedOptimalController.empty;
                elseif isa(hlc, 'PrioritizedController')
                    obj.hlcs = PrioritizedController.empty;
                end
            end
            obj.hlcs(end + 1) = hlc;
        end

    end

    methods (Access = protected)

        function increment_time_step(obj)

            for hlc = obj.hlcs
                hlc.increment_time_step();
            end

        end

        function synchronize_start_with_other_controllers(obj)

            for hlc = obj.hlcs
                hlc.synchronize_start_with_other_controllers();
            end

        end

        function update_controlled_vehicles_traffic_info(obj)

            for hlc = obj.hlcs
                hlc.update_controlled_vehicles_traffic_info();
            end

        end

        function update_hdv_traffic_info(obj)

            for hlc = obj.hlcs
                hlc.update_hdv_traffic_info();
            end

        end

        function create_coupling_graph(obj)

            for hlc = obj.hlcs
                hlc.create_coupling_graph();
            end

        end

        function controller(obj)
            % CONTROLLER Plan trajectory for one time step using a
            % prioritized controller. Vehicles inside one group plan in sequence and
            % between groups plan in parallel. Controller simulates multiple
            % distributed controllers in a for-loop.

            level_matrix = kahn(obj.merged_graph("directed_coupling_sequential"));

            for i_level = 1:size(level_matrix, 1)

                vehicles_in_level = find(level_matrix(i_level, :));

                for i_vehicle = vehicles_in_level
                    obj.hlcs(i_vehicle).controller();
                end

            end

        end

        function result = merged_graph(obj, graph_name)

            result = zeros(length(obj.hlcs), length(obj.hlcs));

            % currently unnecessary as all coupling graphs are the same
            for hlc = obj.hlcs
                result = result | hlc.iter.(graph_name);
            end

        end

        function is_fallback_handled = handle_fallback(obj)

            is_fallback_handled = true;

            for hlc = obj.hlcs
                is_fallback_handled = is_fallback_handled && hlc.handle_fallback();
            end

        end

        function plan_for_fallback(~)
            % is implemented in hlcs
        end

        function store_control_info(obj)

            for hlc = obj.hlcs
                hlc.store_control_info();
            end

        end

        function store_iteration_results(obj)

            for hlc = obj.hlcs
                hlc.store_iteration_results();
            end

        end

        function reset_control_loop_data(obj)

            for hlc = obj.hlcs
                hlc.reset_control_loop_data();
            end

        end

        function apply(obj)

            for hlc = obj.hlcs
                hlc.apply();
            end

        end

        function result = should_stop(obj)
            result = false;

            for hlc = obj.hlcs

                if hlc.should_stop()
                    result = true;
                    return
                end

            end

        end

        function end_run(obj)

            for hlc = obj.hlcs
                hlc.end_run();
                obj.experiment_result{end + 1} = hlc.experiment_result;
            end

        end

        function main_init(obj)

            for hlc = obj.hlcs
                hlc.main_init();
            end

        end

        function synchronize_start_with_plant(obj)

            for hlc = obj.hlcs
                hlc.synchronize_start_with_plant();
            end

        end

        function update_traffic(obj)

            for hlc = obj.hlcs
                hlc.update_traffic();
            end

        end

        function set_run_succeeded(obj, is_run_succeeded)

            for hlc = obj.hlcs
                hlc.set_run_succeeded(is_run_succeeded);
            end

        end

    end

end
