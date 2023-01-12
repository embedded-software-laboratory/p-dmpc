function tests = scenario_unittest
% SCENARIO_UNITTEST    RUN SAMPLE SCENARIOS
    
    tests = functiontests(localfunctions);
end

function testRunScenario1(testcase)

    %load Config from json
    rawJson = fileread('tests/Config_Commonroad_pb_7_visu_non_parl.json');
    options = Config();
    options = options.importFromJson(rawJson);

    main(options);
end

function testRunScenario2(testcase)

    %load Config from json
    rawJson = fileread('tests/Config_Commonroad_pb_2_no_visu_parl.json');
    options = Config();
    options = options.importFromJson(rawJson);

    main(options);
end
