classdef ExperimentResult

    properties
        options Config % config of the simulation

        iteration_data IterationData % iteration steps

        control_results_info ControlResultsInfo

        % indices of vehicles the hlc (belonging to this result) controls; scalar for distributed computation
        hlc_indices

        timing

        git_hash (1, :) char

        file_name (1, :) char % file name without path, without extension
    end

    properties (Dependent)
        n_steps % total number of steps
        t_total % total runtime
        n_hlc % number of hlcs whose experiment results are contained
        max_number_of_computation_levels
    end

    methods

        function obj = ExperimentResult(options, hlc_indices)

            arguments
                options (1, 1) Config;
                hlc_indices (1, :) double;
            end

            obj.options = options;

            obj.timing = cell(0, 1);
            obj.hlc_indices = hlc_indices;
        end

        function equal = is_equal(obj, compare_obj)

            arguments
                obj (1, 1) ExperimentResult;
                compare_obj (1, 1) ExperimentResult;
            end

            equal = obj.n_steps == compare_obj.n_steps;

            for i_step = 1:obj.n_steps
                equal = equal && obj.iteration_data{i_step}.is_equal(compare_obj.iteration_data{i_step});

                if (~equal)
                    return;
                end

            end

        end

        function y_predicted = get_y_predicted(obj, k)

            arguments (Input)
                obj (1, 1) ExperimentResult;
                k (1, 1) double;
            end

            arguments (Output)
                y_predicted (3, :, :) double;
            end

            y_predicted = nan(3, obj.options.Hp, obj.options.amount);
            i_vehicle_start = 1;

            for control_results_info_i = obj.control_results_info(:, k)'
                i_vehicle_end = i_vehicle_start + control_results_info_i.n_vehicles - 1;
                y_predicted(:, :, i_vehicle_start:i_vehicle_end) = control_results_info_i.y_predicted;
                i_vehicle_start = i_vehicle_end + 1;
            end

        end

        function obj = add_meta_information(obj)

            arguments
                obj (1, 1) ExperimentResult;
            end

            [~, git_hash_and_space] = system('git rev-parse --short HEAD');
            obj.git_hash = strtrim(git_hash_and_space);

            [~, obj.file_name, ~] = fileparts( ...
                FileNameConstructor.path_to_experiment_result(obj.options) ...
            );

        end

        function save_merged(obj)

            arguments
                obj (1, 1) ExperimentResult;
            end

            file_path = fullfile( ...
                FileNameConstructor.experiment_result_folder_path(obj.options), ...
                obj.file_name ...
            );
            experiment_result = obj;
            save(file_path, 'experiment_result');
            fprintf('Merged result saved: %s\n', file_path);
        end

        function value = get.n_steps(obj)
            value = length(obj.control_results_info);
        end

        function value = get.t_total(obj)
            value = obj.n_steps * obj.options.dt_seconds;
        end

        function value = get.n_hlc(obj)
            value = length(obj.hlc_indices);
        end

        function value = get.max_number_of_computation_levels(obj)
            value = max([obj.iteration_data.number_of_computation_levels]);
        end

    end

end
