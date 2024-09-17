classdef EvaluationParl
    %EVALUATIONCOMMON Summary of this class goes here
    %   Detailed explanation goes here

    properties
        options % simulation options
        nVeh % number of vehicles
        n_steps % number of time steps
        dt_seconds % sample time
        t_total % total running time
        results_full_path % path where the results are stored

        path_tracking_errors_MAE % mean absolute error
        path_tracking_error_average % average path tracking error

        nodes_expanded_per_step % expanded nodes when graph searching
        nodes_expanded_average % expanded nodes average over steps

        runtime_determine_couplings % runtime of determination of couplings (note that the runtime of reachability analysis is included)
        runtime_determine_couplings_average % average runtime of determination of couplings (note that the runtime of reachability analysis is included)
        runtime_assign_priority % runtime of assignment of priorities
        runtime_assign_priority_average % average runtime of assignment of priorities
        runtime_group_vehs % runtime of grouping vehicles
        runtime_group_vehs_average % average runtime of grouping vehicles
        runtime_graph_search % runtime of graph search
        runtime_graph_search_average % average runtime of graph search
        runtime_total_per_step % total runtime per step
        runtime_total_average % average runtime
        runtime_max % average of maximum top n runtime

        average_speed_each_veh % average speed of each vehicle per step
        average_speed % average speed of all vehicles per step
        max_speed % maximum speed of all vehicles
        min_speed % minimum speed of all vehicles
        sum_speeds_all_vehicles % sum of speeds of all vehicles

        num_couplings % average number of couplings between vehicles
        num_couplings_ignored % average number of couplings reduced by lanelet crossing area
        num_couplings_between_grps % average number of couplings between groups
        num_couplings_between_grps_ignored % average number of couplings between groups reduced by lanelet crossing area

        CLs_num_max % maximum number of actual computation levels

        fallback_rate % fallback rate: fallback_times/n_steps/nVeh
        fallback_steps % number of steps where fallbacks occur

        is_deadlock % true/false, if deadlock occurs
        steps_ignored = 4; % the first several steps are ignored due to their possiblly higher computation time (see MATLAB just-in-time (JIT) compilation)

        belonging_vector % indicates which group each vehicle belongs to
    end

    properties (Access = private)
        reference_paths % reference path of each vehicle
        real_paths % real path of each vehicle
        path_tracking_errors % path tracking errors

        plot_option_real_path
        plot_option_reference_path
        free_flow_speed % free flow speed
        fallback_times % total fallback times of all vehicles. Note that this is not the number of time steps when fallbacks occur
    end

    properties (Dependent)
        mean_tracking_error_per_veh
    end

    methods

        function obj = EvaluationParl(input, T_interval)

            arguments
                input (1, 1) Config
                T_interval (1, :)
            end

            obj.options = input;

            experiment_result = load_latest(obj.options);

            obj.nVeh = experiment_result.options.amount;
            obj.dt_seconds = experiment_result.options.dt_seconds;

            if nargin == 2
                obj.steps_ignored = max(1, floor(T_interval(1) / obj.dt_seconds));
                obj.n_steps = min(floor(T_interval(2) / obj.dt_seconds, length(experiment_result.iteration_data)));
            else
                obj.n_steps = length(experiment_result.iteration_data);
            end

            obj.reference_paths = cell(obj.nVeh, 1);
            obj.real_paths = cell(obj.nVeh, 1);
            obj.path_tracking_errors = cell(obj.nVeh, 1);
            obj.path_tracking_errors_MAE = zeros(obj.nVeh, 1);
            obj.runtime_total_per_step = zeros(0, 1);
            obj.plot_option_real_path = struct(Color = 'b', LineStyle = '-', LineWidth = 1.0, DisplayName = 'Real Path');
            obj.plot_option_reference_path = struct(Color = 'r', LineStyle = '--', LineWidth = 1.0, DisplayName = 'Reference Path');

            obj.n_steps = length(experiment_result.vehicles_fallback);

            obj.t_total = obj.dt_seconds * (obj.n_steps - obj.steps_ignored + 1);
            %             if abs(obj.t_total-result.options.T_end)>obj.dt_seconds
            %                 warning('Simulation stops before reaching the specific time.')
            %             end

            obj = obj.get_path_tracking_errors(experiment_result);
            obj = obj.get_runtime_per_step(experiment_result);
            obj = obj.get_average_speeds;

            % Number of couplings calculations

            for k = 1:experiment_result.n_steps
                % calculate number of couplings
                experiment_result.num_couplings(k) = nnz(experiment_result.iteration_data(k).directed_coupling);
                experiment_result.num_couplings_ignored(k) = nnz(experiment_result.iteration_data(k).directed_coupling) - nnz(experiment_result.iteration_data(k).directed_coupling_reduced);

                % initialize counter
                % number of couplings between parallel groups
                experiment_result.num_couplings_between_grps(k) = 0;
                % reduced number of couplings between groups by using lanelet crossing lanelets
                experiment_result.num_couplings_between_grps_ignored(k) = 0;

                [row_coupling, col_coupling] = find(experiment_result.iteration_data(k).directed_coupling);
                n_couplings = length(row_coupling);

                for i_coupling = 1:n_couplings
                    veh_i = row_coupling(i_coupling);
                    veh_j = col_coupling(i_coupling);
                    veh_ij = [veh_i, veh_j];
                    is_same_grp = any(cellfun(@(c) all(ismember(veh_ij, c)), {experiment_result.iteration_data(k).parl_groups_info.vertices}));

                    if is_same_grp
                        continue
                    end

                    experiment_result.num_couplings_between_grps(k) = experiment_result.num_couplings_between_grps(k) + 1;

                    if ( ...
                            ~(experiment_result.iteration_data(k).directed_coupling(veh_i, veh_j) == 1) || ...
                            ~(experiment_result.iteration_data(k).directed_coupling_reduced(veh_i, veh_j) == 0) ...
                        )
                        continue
                    end

                    experiment_result.num_couplings_between_grps_ignored(k) = experiment_result.num_couplings_between_grps_ignored(k) + 1;
                end

            end

            % Number of couplings
            obj.num_couplings = mean(cellfun(@(e) nnz(e.directed_coupling), experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)));
            obj.num_couplings_ignored = mean(cellfun(@(e) nnz(e.directed_coupling) - nnz(e.directed_coupling_reduced), experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)));
            obj.num_couplings_between_grps = mean(cellfun(@(e) e.num_couplings_between_grps, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)));
            obj.num_couplings_between_grps_ignored = mean(cellfun(@(e) e.num_couplings_between_grps_ignored, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)));

            obj.CLs_num_max = max(experiment_result.computation_levels(obj.steps_ignored:obj.n_steps));

            obj.belonging_vector = cell2mat(cellfun(@(e) e.belonging_vector(:), experiment_result.iteration_data, UniformOutput = false));

            obj.fallback_times = 0;
            obj.fallback_steps = 0;

            for i = obj.steps_ignored:obj.n_steps

                if ~isempty(experiment_result.vehicles_fallback{i})
                    obj.fallback_times = obj.fallback_times + length(experiment_result.vehicles_fallback{i});
                    obj.fallback_steps = obj.fallback_steps + 1;
                end

            end

            obj.fallback_rate = obj.fallback_times / (obj.n_steps - obj.steps_ignored + 1) / obj.nVeh;

            obj.is_deadlock = is_deadlock(experiment_result);

        end

        function obj = get_path_tracking_errors(obj, experiment_result)

            arguments
                obj (1, 1) EvaluationParl;
                experiment_result (1, 1) ExperimentResult;
            end

            % Calculate the path tracking error
            for iVeh = 1:obj.nVeh

                for iIter = obj.steps_ignored:obj.n_steps
                    i = iIter - obj.steps_ignored + 1;
                    obj.reference_paths{iVeh}(:, i) = reshape(experiment_result.iteration_data(iIter).reference_trajectory_points(iVeh, 1, :), 2, []);
                    obj.real_paths{iVeh}(:, i) = experiment_result.vehicle_path_fullres{iVeh, iIter}(end, 1:2)'; % FIXME
                    obj.path_tracking_errors{iVeh}(:, i) = sqrt(sum((obj.real_paths{iVeh}(:, i) - obj.reference_paths{iVeh}(:, i)).^2, 1));
                end

                obj.path_tracking_errors_MAE(iVeh) = mean(obj.path_tracking_errors{iVeh});
            end

            obj.path_tracking_error_average = mean(obj.path_tracking_errors_MAE);

            %             obj.visualize_path(result)
        end

        function obj = get_runtime_per_step(obj, experiment_result)

            arguments
                obj (1, 1) EvaluationParl;
                experiment_result (1, 1) ExperimentResult;
            end

            % Calculate the total runtime per step
            assert(abs(experiment_result.n_steps - length(experiment_result.timing.traffic_situation_update)) <= 1)
            %TODO: runtime_subcontroller_max can be calculated from 'controller' field in timings
            obj.runtime_total_per_step = experiment_result.timing.traffic_situation_update(obj.steps_ignored:obj.n_steps)' + experiment_result.runtime_subcontroller_max(obj.steps_ignored:obj.n_steps)';

            % Find outliers
            outliers = find(isoutlier(obj.runtime_total_per_step));
            disp(['Find ' num2str(length(outliers)) ' outliers.'])
            %             % set outliers to average values
            %             obj.runtime_total_per_step(outliers) = obj.runtime_average;
            % ignore the first several steps as they may have higher values
            % in computation time due to the just-in-time (JIT) compilation

            obj.runtime_total_average = sum(obj.runtime_total_per_step) / (obj.n_steps - obj.steps_ignored + 1);

            n_max = 1;
            obj.runtime_max = mean(maxk(obj.runtime_total_per_step, n_max));

            %TODO: graph search can be calculated from 'optimizer' field in timings
            obj.runtime_graph_search_average = mean(experiment_result.runtime_graph_search_max(obj.steps_ignored:obj.n_steps));
            obj.runtime_graph_search = max(experiment_result.runtime_graph_search_max(obj.steps_ignored:obj.n_steps));

            %TODO: add coupling time values to list and then perform operation

            obj.runtime_determine_couplings = max(cellfun(@(e) e.timer.determine_couplings, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)) + experiment_result.timing.traffic_situation_update(obj.steps_ignored:obj.n_steps));
            obj.runtime_determine_couplings_average = mean(cellfun(@(e) e.timer.determine_couplings, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)) + experiment_result.timing.traffic_situation_update(obj.steps_ignored:obj.n_steps));
            % in order to assign priorities, determination of couplings is needed
            obj.runtime_assign_priority = max(cellfun(@(e) e.timer.assign_priority, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)) + experiment_result.timing.traffic_situation_update(obj.steps_ignored:obj.n_steps));
            obj.runtime_assign_priority_average = mean(cellfun(@(e) e.timer.assign_priority, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)) + experiment_result.timing.traffic_situation_update(obj.steps_ignored:obj.n_steps));
            obj.runtime_group_vehs = max(cellfun(@(e) e.timer.group_vehs, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)));
            obj.runtime_group_vehs_average = mean(cellfun(@(e) e.timer.group_vehs, experiment_result.iteration_data(obj.steps_ignored:obj.n_steps)));

            obj.nodes_expanded_per_step = experiment_result.n_expanded(obj.steps_ignored:obj.n_steps);
            obj.nodes_expanded_average = mean(obj.nodes_expanded_per_step);
        end

        function obj = get_average_speeds(obj)
            % Calculates the average speed of each vehicle
            % note that we use reference path but not real path
            reference_paths_diff = cellfun(@(c) diff(c, 1, 2), obj.reference_paths, UniformOutput = false);
            distances = cellfun(@(c) sum(sqrt(c(1, :).^2 + c(2, :).^2)), reference_paths_diff);
            obj.average_speed_each_veh = distances ./ obj.t_total;
            obj.average_speed = mean(obj.average_speed_each_veh);
            obj.min_speed = min(obj.average_speed_each_veh);
            obj.max_speed = max(obj.average_speed_each_veh);
            obj.sum_speeds_all_vehicles = sum(obj.average_speed_each_veh);

            %             % get sampleTime_FFS
            %             [~,obj.free_flow_speed] = FreeFlowSpeed;
            %             if options.coupling ~= CouplingStrategies.no_coupling
            %                 obj.travel_time_index = obj.average_speed/obj.free_flow_speed(num2str(obj.dt_seconds));
            %             else
            %                 obj.travel_time_index = 1;
            %             end
        end

        function visualize_path(obj, experiment_result)

            arguments
                obj (1, 1) EvaluationParl;
                experiment_result (1, 1) ExperimentResult;
            end

            % visualize reference path and real path
            figure()
            hold on
            box on
            axis equal

            xlabel('\fontsize{14}{0}$x$ [m]', Interpreter = 'LaTex');
            ylabel('\fontsize{14}{0}$y$ [m]', Interpreter = 'LaTex');

            xlim(experiment_result.options.plot_limits(1, :));
            ylim(experiment_result.options.plot_limits(2, :));
            daspect([1 1 1])

            plot_lanelets(experiment_result.scenario.road_raw_data.lanelet)

            % get vehicle that has maximum path tracking error
            [~, vehs_max_error] = maxk(obj.path_tracking_errors_MAE, 5);
            % get vehicle that has minimum path tracking error
            [~, vehs_min_error] = mink(obj.path_tracking_errors_MAE, 5);

            veh_max_error = vehs_max_error(1);
            veh_min_error = vehs_min_error(1);
            s_max = obj.plot_path(obj.real_paths{veh_max_error}, obj.path_tracking_errors{veh_max_error});

            c = colorbar;
            c.Title.String = 'Path Tracking Error';
            p_max = plot(obj.reference_paths{veh_max_error}(1, :), obj.reference_paths{veh_max_error}(2, :), obj.plot_option_reference_path);
            legend([s_max, p_max], {'Real Path (max error)', 'Reference Path (max error)'})
            %             s_min = obj.plot_path(obj.real_paths{veh_min_error},obj.path_tracking_errors{veh_min_error});
            %             p_min = plot(obj.reference_paths{veh_min_error}(1,:),obj.reference_paths{veh_min_error}(2,:),obj.plot_option_reference_path);
            %             legend([s_max,s_min,p_max,p_min],{'Real Path (max error)','Real Path (min error)','Reference Path (max error)','Reference Path (min error)'})
        end

        function s = plot_path(~, path, error)
            x = path(1, :); y = path(2, :);
            z = zeros(size(x));
            lineColor = error;
            s = surface([x; x], [y; y], [z; z], [lineColor; lineColor], FaceColor = 'no', EdgeColor = 'interp', LineWidth = 4);
        end

    end

    methods (Static)

        function save_fig(fig, file_name)
            % save fig to pdf
            %             set(fig,'Units','centimeters');
            %             pos = get(fig,'Position');
            %             set(fig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

            [file_path, ~, ~] = fileparts(mfilename('fullpath'));
            folder_target = fullfile(file_path, 'fig');

            if ~isfolder(folder_target)
                % create target folder if not exist
                mkdir(folder_target)
            end

            file_name_svg = [file_name '.svg'];
            file_name_pdf = [file_name '.pdf'];
            full_path_svg = fullfile(folder_target, file_name_svg);
            full_path_pdf = fullfile(folder_target, file_name_pdf);

            print(fig, full_path_pdf, '-dpdf', '-r0');
            print(fig, full_path_svg, '-dsvg', '-r0');

        end

    end

end
