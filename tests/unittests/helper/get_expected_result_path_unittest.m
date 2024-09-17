function tests = get_expected_result_path_unittest
    % get_expected_result_path_unittest Test wether the GET_EXPECTED_RESULT_PATH function produces
    %                           correct paths for expected results given a result path

    tests = functiontests(localfunctions);
end

function test_contains_subpath(testCase)
    path = 'graph_based_planning/results/circle_par-rhgs/type_realistic_Hp6_dt0.2_nVeh3_T4_full_coupling_coloring_priority_maxCLs2_constraint-from-successor-area_of_standstill_EnterLaneletCrossingArea1_Wdistance_weight.mat';

    expected_path = get_expected_result_path(path);

    testCase.verifyTrue(contains(expected_path, 'tests/systemtests/expected_results'), "expected result's path is wrong");
end

function test_equals_expected_path(testCase)
    path = 'graph_based_planning/results/circle_par-rhgs/type_realistic_Hp6_dt0.2_nVeh3_T4_full_coupling_coloring_priority_maxCLs2_constraint-from-successor-area_of_standstill_EnterLaneletCrossingArea1_Wdistance_weight.mat';
    expected_path = 'graph_based_planning/tests/systemtests/expected_results/circle_par-rhgs/type_realistic_Hp6_dt0.2_nVeh3_T4_full_coupling_coloring_priority_maxCLs2_constraint-from-successor-area_of_standstill_EnterLaneletCrossingArea1_Wdistance_weight.mat';

    actual_expected_path = get_expected_result_path(path);

    testCase.verifyTrue(strcmp(actual_expected_path, expected_path), sprintf("expected: %s\n actual: %s", expected_path, actual_expected_path));
end
