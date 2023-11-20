classdef FileNameConstructor
    % FILENAMECONSTRUCTOR This class provides methods to construct
    % different file names according to certain rules.

    properties

    end

    methods

        function obj = FileNameConstructor()
        end

    end

    methods

    end

    methods (Static)

        function controller_name = get_controller_name(options)

            if options.is_prioritized

                if options.compute_in_parallel
                    controller_name = strcat('par. PB-', 'RHGS-', char(options.priority));
                else
                    controller_name = strcat('seq. PB-', 'RHGS-', char(options.priority));
                end

            else
                controller_name = strcat('centralized-', 'RHGS-', char(options.priority));
            end

        end

        function mpa_instance_name = get_mpa_name(options)

            arguments
                options (1, 1) Config
            end

            % GET_MPA_NAME Construct name for the file to which a object of the
            % class MPA is saved.
            % Example:  MPA_type_single_speed_Hp6
            %           MPA_type_triple_speed_Hp6_parl_non-convex

            mpa_instance_name = ['MPA_', 'type_', char(options.mpa_type), '_Hp', num2str(options.Hp), '_T', num2str(options.dt_seconds)];

            if options.is_allow_non_convex
                mpa_instance_name = [mpa_instance_name, '_non-convex'];
            end

            if ~options.is_prioritized
                mpa_instance_name = [mpa_instance_name, '_centralized_nVeh' num2str(options.amount)];
            end

            if options.is_use_dynamic_programming
                mpa_instance_name = [mpa_instance_name, '_DP'];
            end

            if ~options.recursive_feasibility
                mpa_instance_name = [mpa_instance_name, '_notRecFeas'];
            end

            mpa_instance_name = [mpa_instance_name, '.mat'];
        end

        function results_folder_path = gen_results_folder_path(options)

            if options.is_prioritized
                controller_name = 'par-rhgs';
            else
                controller_name = 'cen-rhgs';
            end

            results_folder_name = strrep(strcat(char(options.scenario_type), '_', controller_name), ' ', '_');

            results_folder_path = fullfile( ...
                FileNameConstructor.all_results(), results_folder_name ...
            );

            if ~isfolder(results_folder_path)
                % create target folder if not exist
                mkdir(results_folder_path)
            end

        end

        function video_file_path = gen_video_file_path(options)
            video_file_name = [ ...
                                   'video_', ...
                                   FileNameConstructor.gen_scenario_name(options), ...
                                   '.avi' ...
                               ];
            video_file_path = fullfile( ...
                FileNameConstructor.gen_results_folder_path(options) ...
                , video_file_name ...
            );
        end

        function scenario_name = gen_scenario_name(options, i_vehicles)

            arguments
                options (1, 1) Config;
                i_vehicles (1, :) {mustBeInteger, mustBePositive} = options.amount;
            end

            priority = char(options.priority);
            weight = char(options.weight);

            scenario_name = '';

            if options.compute_in_parallel
                scenario_name = ['veh_', num2str(options.path_ids(i_vehicles)), '_'];
            end

            if isempty(options.result_name)
                % use default name
                scenario_name = [scenario_name, 'type_', char(options.mpa_type), '_Hp', num2str(options.Hp), '_dt', num2str(options.dt_seconds), '_nVeh', num2str(options.amount), '_T', num2str(options.T_end), '_', char(options.coupling), '_', priority];

                veh_ids_str = sprintf('-%d', options.path_ids);
                scenario_name = [scenario_name, '_ids', veh_ids_str];

                if options.is_prioritized
                    scenario_name = [scenario_name, '_maxCLs', num2str(options.max_num_CLs), ...
                                         '_ConsiderVehWithoutROW', options.strategy_consider_veh_without_ROW, '_EnterLaneletCrossingArea', options.strategy_enter_lanelet_crossing_area];
                end

                if options.fallback_type ~= FallbackType.local_fallback
                    % local fallback is the default fallback strategy
                    scenario_name = [scenario_name, '_', char(options.fallback_type)];
                end

                if ~options.should_reduce_result
                    scenario_name = [scenario_name, '_fullResult'];
                end

                if ~options.isDealPredictionInconsistency
                    scenario_name = [scenario_name, '_notDealWithPredictionInconsistency'];
                end

                if ~(options.weight == WeightStrategies.STAC_weight)
                    scenario_name = [scenario_name, '_W', weight];
                end

                if ~options.is_bounded_reachable_set_used
                    scenario_name = [scenario_name, '_unboundedRS'];
                end

            else
                % use custom name
                scenario_name = [scenario_name, options.result_name];
            end

        end

        function results_full_path = get_results_full_path(options, i_vehicles)
            % GET_RESULTS_FULL_PATH Construct name for the folder where simulation
            % results are saved.
            % INPUT: options, i_vehicles(vehicles for which this HLC is responsible.)
            results_name = [FileNameConstructor.gen_scenario_name(options, i_vehicles), '.mat'];

            results_full_path = fullfile( ...
                FileNameConstructor.gen_results_folder_path(options) ...
                , results_name ...
            );
        end

        function folder_path = all_results()
            [file_path, ~, ~] = fileparts(mfilename('fullpath')); % get the path of the current file
            idcs = strfind(file_path, filesep); % find all positions of '/'
            main_folder = file_path(1:idcs(end) - 1); % one folder up
            folder_path = fullfile(main_folder, 'results');

            if ~isfolder(folder_path)
                mkdir(folder_path)
            end

        end

    end

end
