### Create a Job

- Go to gitlab-ci.yml
- Add a Job:

```
`JobName:
  extends: .matlab_defaults
  stage: Test
  script:
    - matlab -batch "startup(); results = runtests('tests/myTest.m'); assertSuccess(results);"
  tags:
    - docker
```



### Setup Matlab Continuous Integration
#### Server Requirements
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


#### Create Matlab Docker image as required
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
