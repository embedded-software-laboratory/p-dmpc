function tests = scenario_unittest
% SCENARIO_UNITTEST    RUN SAMPLE SCENARIOS
    
    tests = functiontests(localfunctions);
end

function testRunScenario1(testcase)

    %load Config from json
    rawJson = fileread('tests/ConfigScenario.json');
    options = Config();
    options = options.importFromJson(rawJson);

    main(options);
end
