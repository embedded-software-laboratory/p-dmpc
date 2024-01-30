classdef FileNameConstructor
    % FILENAMECONSTRUCTOR This class provides methods to construct
    % different file names according to certain rules.

    methods

        function obj = FileNameConstructor()
        end

    end

    methods (Static)

        function mpa_instance_name = get_mpa_name(options)

            arguments
                options (1, 1) Config
            end

            % GET_MPA_NAME Construct name for the file to which a object of the
            % class MPA is saved.
            % Example:  MPA_type_single_speed_Hp6
            %           MPA_type_triple_speed_Hp6_parl_non-convex

            mpa_instance_name = [char(options.mpa_type), '_Hp-', num2str(options.Hp), '_T-', num2str(options.dt_seconds)];

            if ~options.is_prioritized
                mpa_instance_name = [mpa_instance_name, '_centralized'];
            end

            mpa_instance_name = [mpa_instance_name, '.mat'];
        end

        function folder_path = experiment_result_folder_path(options)

            arguments
                options (1, 1) Config
            end

            folder_path = fullfile( ...
                FileNameConstructor.all_results(), ...
                char(options.scenario_type), ...
                sprintf("%02d", options.amount) ...
            );

            if options.is_prioritized
                folder_path = fullfile(folder_path, char(options.priority));
            end

            if ~isfolder(folder_path)
                % create target folder if not exist
                mkdir(folder_path)
            end

        end

        function file_path = path_to_accompanying_file( ...
                experiment_result, ...
                suffix_and_extension ...
            )

            arguments
                experiment_result (1, 1) ExperimentResult;
                suffix_and_extension (1, :) char;
            end

            file_path = fullfile( ...
                FileNameConstructor.experiment_result_folder_path( ...
                experiment_result.options ...
            ), ...
                strcat(experiment_result.file_name, "_", suffix_and_extension) ...
            );

        end

        function file_path = path_to_experiment_result(options)

            arguments
                options (1, 1) Config;
            end

            results_name = strcat(string(datetime("now", Format = "yyMMdd-HHmm")), ".mat");

            file_path = fullfile( ...
                FileNameConstructor.experiment_result_folder_path(options) ...
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

        function folder_path = temporary_sub_results_folder_path()

            folder_path = fullfile( ...
                FileNameConstructor.all_results(), ...
                'tmp' ...
            );

            if ~isfolder(folder_path)
                mkdir(folder_path)
            end

        end

        function file_path = path_to_temporary_sub_result(experiment_result)

            arguments
                experiment_result (1, 1) ExperimentResult;
            end

            file_path = fullfile( ...
                FileNameConstructor.temporary_sub_results_folder_path(), ...
                sprintf("%02d", experiment_result.hlc_indices) ...
            );

        end

        function experiment_results = load_all(options)

            arguments (Input)
                options (1, 1) Config
            end

            arguments (Output)
                experiment_results (1, :) ExperimentResult
            end

            experiment_results = ExperimentResult.empty(0, 0);

            result_folder = FileNameConstructor.experiment_result_folder_path(options);

            files = dir(fullfile(result_folder, "*.mat"));

            for i = 1:length(files)
                result = load(fullfile(result_folder, files(i).name)).experiment_result;

                if isequal(options, result.options)
                    experiment_results(length(experiment_results) + 1) = result;
                end

            end

        end

        function experiment_result = load_latest(options)

            arguments
                options (1, 1) Config;
            end

            arguments (Output)
                experiment_result (1, 1) ExperimentResult
            end

            % see #243 getLatest(options) function specification
        end

        function exists = result_exists(options)

            arguments
                options (1, 1) Config;
            end

            exists = ~isempty(FileNameConstructor.load_all(options));
        end

    end

end
