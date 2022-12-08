function tests = scenario_unittest
% SCENARIO_UNITTEST    RUN SAMPLE SCENARIOS
    
    tests = functiontests(localfunctions);
end

function testRunScenario1(testcase)

    %load Config from json
    rawJson = fileread('tests/ConfigScenario.json');
    options = Config();
    options = options.importFromJson(rawJson);

    %random seed for reproducibility
    random_seed = RandStream('mt19937ar');

    %create scenario
    scenario = create_scenario(options, random_seed);

    %run scenario
    run_scenario(scenario);
end
