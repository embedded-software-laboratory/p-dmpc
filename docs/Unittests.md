### Where to add Tests?
Please add all tests in the folder tests.

Add tests testing a single function into the unittests folder.
Add tests testing the whole system starting from the ```main```-function into the systemtests folder

### How to run a custom scenario as a systemtest?
- Create a Config.json with the desired options set. You can find an example in ```tests/systemtests/Config_systemtests.json```.
- Create a Config object from your json. You can find an example in ```tests/systemtests/systemtests.m```.
- Call the ```main```-function with your configuration.

### How to create parameterized tests?
Matlab allows to create test classes with properties whose values are iterated for all tests using these properties.
You can find an example in ```tests/systemtests/systemtests.m```.
Only string and integer values are printed in the test summary, so it is best to use only those types for iterated parameters to have a meaningful test summary.
See the [Matlab Documentation](https://de.mathworks.com/help/matlab/matlab_prog/create-basic-parameterized-test.html) to learn more.

You can also write advanced parameterized tests with Matlab which allows for [different strategies to iterate the parameters](https://de.mathworks.com/help/matlab/matlab_prog/create-advanced-parameterized-test.html) and also allows to use [external data for parameter values](https://de.mathworks.com/help/matlab/matlab_prog/use-external-parameters-in-parameterized-test.html).

### How to make a test count for code coverage?
Add the test file to a job extending the job .matlab_with_test_coverage by setting the test folder as the variable ```$TEST_FOLDER``` and by setting the tested folder(s) as the variable ```$COVERAGE_FOLDER```.