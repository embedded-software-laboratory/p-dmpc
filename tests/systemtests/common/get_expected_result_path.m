function expected_result_path = get_expected_result_path(output_path)

    arguments
        output_path (1, 1) string;
    end

    expected_result_path = regexprep(output_path, 'results', 'tests/systemtests/expected_results', "once");
end
