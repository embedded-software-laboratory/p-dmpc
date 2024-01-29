function results = load_all(options)

    arguments
        options (1, 1) Config
    end

    results = ExperimentResult.empty(0, 0);

    result_folder = FileNameConstructor.experiment_result_folder_path(options);

    files = dir(fullfile(result_folder, "*.mat"));

    for i = 1:length(files)
        result = load(fullfile(result_folder, files(i).name)).experiment_result;

        if isequal(options, result.options)
            results(length(results) + 1) = result;
        end

    end

end
