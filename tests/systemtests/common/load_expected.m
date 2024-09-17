function expected_result = load_expected(experiment_result)

    arguments
        experiment_result (1, 1) ExperimentResult;
    end

    output_path = FileNameConstructor.experiment_result_folder_path(experiment_result.options);
    expected_result_output_path = get_expected_result_path(output_path);

    expected_result = FileNameConstructor.load_latest(experiment_result.options, result_folder_path = expected_result_output_path);

end
