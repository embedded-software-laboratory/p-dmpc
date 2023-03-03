### Where to add Tests?
Please add all tests in the folder tests.

### How to run a custom scenario as a test?
- Create a Config.json with the desired options set. You can find an example in ```tests/ConfigScenario.json```.
- Create a Config object from your json. You can find an example in ```tests/scenario_unittest.m```.
- Call the run_scenario function with your configuration.

### How to make a test count for code coverage?
Add the test file to a job extending the job .matlab_with_coverage by appending the test file path to the variable ```$TEST_FILES``` and by appending the tested folder(s) to the variable ```$COVERAGE_FOLDERS```.