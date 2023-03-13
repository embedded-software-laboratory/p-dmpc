# Create a Job

- Go to gitlab-ci.yml
- Add a Job or extend another job

### Job for a test *with* code coverage and test summary collection:
```
JobName:
  extends: .matlab_with_test_coverage
  variables:
    TEST_FOLDER: "'folder'"
    COVERAGE_FOLDER: "{'folder1', 'folder2', ..., 'folderm'}"
```
As the overall code coverage value is computed as the average of all job coverages, tests for the same folders should be collected in one job to get a meaningful overall coverage value.

### Job for a test *with* test summary, but *without* code coverage data collection:
```
JobName:
  extends: .matlab_with_test_summary
  variables:
    TEST_FOLDER: "'folder'"
```
Test summary data collection should be used by default as it allows a nice overview of failed tests in Gitlab.

### Job for a test *without* code coverage or test summary data collection:
```
`JobName:
  extends: .matlab_defaults
  stage: Test
  script:
    - matlab -batch "startup(); results = runtests('tests/myTest.m'); assertSuccess(results);"
  tags:
    - docker
```

### Parallel jobs
A set of jobs with different values can be executed in parallel via the ```parallel matrix``` keyword in the .gitlab-ci.yaml.
To really run in parallel, the Gitlab runner has to be configured to allow parallel execution which is currently not the case.
But also without parallel execution, using this strategy provides a compact and systematic way to generate tests. This is currently used for the ```Unittests``` job which can be used as an example.

See the [Gilab CI documentation](https://docs.gitlab.com/ee/ci/yaml/#parallel) for more information.


# Helper Tools
## JUnit Test Summary
JUnit Test Summaries are visualized in a Gitlab CI pipeline as an overview of failed and passed tests so that it can easily identified which tests errored and why.

There already exists a .gitlab-ci.yaml job template ```.matlab_with_test_summary``` to execute Matlab tests and create and upload a JUnit test summary. See [the section about creating jobs above](#job-for-a-test-with-test-summary-but-without-code-coverage-data-collection).

To learn how to use JUnit test summaries together with Gitlab CI, look at the [Gitlab Documentation](https://docs.gitlab.com/ee/ci/testing/unit_test_reports.html).

To learn how to create a JUnit test summary in Matlab, read the [Matlab Documentation](https://de.mathworks.com/help/matlab/ref/matlab.unittest.plugins.xmlplugin-class.html).

## Cobertura Code Coverage report
Cobertura XML is a code coverage report format that is recognized by Gitlab CI. Such a report can be uploaded as a job artifact from a CI pipeline and is then visualized in the job summary, on the main page in a badge and in a merge request.

There already exists a .gitlab-ci.yaml job template ```.matlab_with_test_coverage``` to execute Matlab tests and create and upload a Cobertura code coverage report. See [the section about creating jobs above](#job-for-a-test-with-code-coverage-and-test-summary-collection).

To learn how to use Cobertura code coverage reports together with Gitlab CI, look at the [Gitlab Documentation](https://docs.gitlab.com/ee/ci/testing/unit_test_reports.htmlhttps://de.mathworks.com/help/matlab/ref/matlab.unittest.plugins.codecoverage.coberturaformat-class.htmlhttps://docs.gitlab.com/ee/ci/testing/test_coverage_visualization.html).

To learn how to create a Cobertura code coverage report in Matlab, read the [Matlab Documentation](https://de.mathworks.com/help/matlab/ref/matlab.unittest.plugins.xmlplugin-class.htmlhttps://de.mathworks.com/help/matlab/ref/matlab.unittest.plugins.codecoverage.coberturaformat-class.html).


# Setup Matlab Continuous Integration
## Server Requirements
-	Install Docker
-	Make sure Docker is running and started automatically during system boot
-	Install gitlab runner (for Linux) Install GitLab Runner using the official GitLab repositories | GitLab
	-	curl -L "https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.deb.sh" | sudo bash
	-	sudo apt-get install gitlab-runner
-	Register the runner Registering runners | GitLab
	-	sudo gitlab-runner register
	-	tags=docker
	-	executor=docker
-	Start Runner


## Create Matlab Docker image
### Manually create image
Create individual Matlab Docker image (with Python, gcc, ROS):
-	Create container based on mathworks/matlab docker image. Use "docker run -it  --name MyMatlab -p 8888:8888 -p 6080:6080 -p 5901:5901 --shm-size=512M mathworks/matlab:r2022a -vnc" as a base container
-	Access the container via your browser http://hostname:6080
-	Execute matlab as root
-	Activate matlab using your license or a license server( Set env variable MLM_LICENSE_FILE to use a license server)
-	Install ros toolbox, parallel computing toolbox, Statistics and Machine Learning Toolbox
-	Install python 3.9,  cmake, gcc
-	Remove your matlab license (Navigate to matlab folder and delete license files)
-	docker commit containerID  yourNewImage to build you own Matlab Image based on the modified container
-	Tag image like your gitlab registry (docker tag SOURCE_IMAGE[:TAG] TARGET_IMAGE[:TAG])
-	Push image to gitlab container registry

### Automatically create image
A Dockerfile exists to install Matlab in a clean Ubuntu 18.04. Docker image. It was tested for Ubuntu 18.04, Matlab R2022a and the corresponding ROS Toolbox dependencies. _For other versions small changes might be necessary_.
- Set variables in ```SETTINGS``` section of the install script ```docker/install.sh```.
- Execute script ```./docker/install.sh```.
A more detailed documentation can be found in ```docker/readme.md```.
