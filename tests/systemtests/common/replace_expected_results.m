function replace_expected_results()
    scenarios = [ScenarioType.circle, ScenarioType.commonroad, ScenarioType.lanelet2];

    results_dir = FileNameConstructor.all_results();
    expected_results_dir = get_expected_result_path(results_dir);

    for scenario = scenarios
        from_dir = strcat(results_dir, '/', string(scenario));
        to_dir = strcat(expected_results_dir, '/', string(scenario));
        fprintf(sprintf("copying content of %s to %s...", from_dir, to_dir))
        copyfile(from_dir, to_dir);
        fprintf("done.\n");
    end

    files = dir(strcat(expected_results_dir, '/**'));

    for i = 1:numel(files)

        if ~files(i).isdir && ~contains(files(i).name, ".mat")
            delete(strcat(files(i).folder, '/', files(i).name));
        end

    end

end
