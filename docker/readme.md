# Generate Docker Image for CI

The purpose of the files in the folder `docker` is to automatically generate a Docker image for the Gitlab CI. This is necessary when something in the environment (Matlab or Linux version) changes. The purpose of each file is:

- [`install.sh`](#install-script): has to be executed to automatically build the new docker image and push it to the Gitlab Container Registry. Before, [some variables have to be set](#variable-settings) in order to work correctly.
- [`Dockerfile`](#dockerfile): contains the description how to generate the docker image, it is used by the install.sh script. Here, no changes should be necessary. The only exception is when another Ubuntu base image is used as then the available, needed apt packages might differ.

The content of the two files is described in more detail in the following two sections. Further, it is described how to use the resulting Docker image for [CI](#ci).

## Dockerfile

The Dockerfile was inspired by [Instructions from Mathworks to create a MATLAB Container Image](https://github.com/mathworks-ref-arch/matlab-dockerfile). The important steps are described in the following sub-sections. To find out more about the different used Docker commands look at the [Dockerfile reference](https://docs.docker.com/engine/reference/builder/).

### Docker Base Image

Mathworks provides Docker images with pre-installed MATLAB dependencies at [Docker Hub](https://hub.docker.com/r/mathworks/matlab-deps/tags). But to make the install process more transparent, it was decided to base the Docker image onto a clean Ubuntu image so that every dependency has to be installed by yourself.

### MATLAB Dependencies

MATLAB itself has very little dependencies, but for the ROS Toolbox, a correct python and gcc version are needed. Therefore, check the [current dependencies](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html).

Further, for Ubuntu 18.04 some changes had to be made in order to work correctly, namely updating `libstdc++6`, installing `libxt6` and `libxext6` as well as specifying the path to `libGL.so.1`.

### MATLAB Package Manager (MPM)

To install MATLAB upon the base image, the [MATLAB Package Manager (MPM)](https://de.mathworks.com/matlabcentral/fileexchange/54548-mpm) is used. Therefore, first some dependencies have to be installed via `apt-get`. Then the MPM software can be downloaded via `wget`. Calling `./mpm install` then installs Matlab with a specified release and toolboxes. Finally, the MATLAB installation is linked to `/usr/local/bin` to be executable via `matlab` from command-line and MPM is removed.

## Install Script

The install script consists of two sections:

1. SETTINGS: where some variables have to be set before execution and others can be changed.
2. SCRIPT: where docker build, login and push is executed. Here, no changes should be necessary.

### Variable Settings

If you want to push the created docker image to the Gitlab container registry, set the variable `PUSH_IMAGE` to `true`. Default is `false`.
Then, the following settings have to be set:

- `GITLAB_USERNAME`: Set to your MATLAB username.
- `GITLAB_ACCESS_TOKEN`: Set to your access token to log into the Gitlab Container Registry. The access token needs to have the scopes `read_registry` and `write_registry` enabled. If you do not have such a personal access token, generate one as described in the [Gitlab Documentation](https://docs.gitlab.com/ee/user/profile/personal_access_tokens.html#create-a-personal-access-token).

Furthermore, the following settings can be changed:

- `MATLAB_RELEASE`: Set to the MATLAB release to install. Default is `r2022a`. Use only lower case.
- `UBUNTU_VERSION`: Set the Ubuntu version to use as a base image. Check the possible tags at [Docker Hub](https://hub.docker.com/_/ubuntu). Default is `18.04`.
- `PYTHON_VERSION` and `GCC_VERSION`: Set the python and gcc version needed for the [MATLAB ROS Toolbox](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html). Specify subversions for python, but not for gcc. Default is python `3.9.6` and gcc `6`.
- `TOOLBOXES`: Set to a spaces separated list of toolboxes to be installed with MATLAB. Make sure to replace spaces in the Toolbox names by underscores. Default is `"Parallel_Computing_Toolbox ROS_Toolbox"`.
- `LICENSE_SERVER`: Set to the network license server address for MATLAB. The value should be of the format <port>@<host>. Default is `50022@license3.rz.rwth-aachen.de`.
- `IMAGE_NAME`: Set to the name of the Docker image pushed to the Gitlab container registry. Default is `matlab_with_ros`.
- `IMAGE_TAG`: Set to the tag of the docker image pushed to the Gitlab container registry. Default is `latest`.

## CI

To use the built and pushed Docker image for Continuous Integration, the name of the container registry simply has to be specified in the `.gitlab-ci.yml`:

Specify a job from an image as follows:

    <job_name>:
      image:
        name: registry.git-ce.rwth-aachen.de/cpm/coincar/software/graph_based_planning/<IMAGE_NAME>:<IMAGE_TAG>
        entrypoint: [""]
