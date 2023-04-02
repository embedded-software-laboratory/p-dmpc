#!/bin/bash

# --------------------------------------------- SETTINGS ----------------------------------------------------

# All settings should be possible to set in this SETTINGS section. The SCRIPT section shouldn't have to be changed.

# Define whether the built image should be pushed to the Gitlab container registry by setting PUSH_IMAGE to true or false. Default is false.
# If set to true, the following environment variables GITLAB_USERNAME and GITLAB_ACCESS_TOKEN have to be set.
# If you first only build the image and then decide to push, just rerun the script with PUSH_IMAGE=true and the image does not have to be rebuild due to docker's cache.
PUSH_IMAGE=false

# Credentials to log into the Gitlab container registry can be set in the variables GITLAB_USERNAME and GITLAB_ACCESS_TOKEN.
# To generate such a personal access token see https://docs.gitlab.com/ee/user/profile/personal_access_tokens.html#create-a-personal-access-token.
# The access token needs to have the scopes read_registry and write_registry enabled.
GITLAB_USERNAME=<username>
GITLAB_ACCESS_TOKEN=<access_token>

# To install another MATLAB release than R2022a in the container, change the value of MATLAB_RELEASE. Use only lower case.
MATLAB_RELEASE=r2022a

# To change the Ubuntu version to use as OS for the container, change the value of UBUNTU_VERSION. Standard is Ubuntu 18.04 as this is the version
# installed in the CPM Lab. Other possible tags can be found under https://hub.docker.com/_/ubuntu.
# Caution: If another Ubuntu version is used, the Dockerfile might have to be adjusted due to other available apt packages.
UBUNTU_VERSION=18.04

# To change the Python CMake or GCC version needed for ROS Toolbox, change the value of PYTHON_VERSION, CMAKE_VERSION resp. GCC_VERSION.
# Default is Python 3.9.6, CMake 3.16.3 and GCC 6.3 which fulfill the requirements for MATLAB R2022a.
# Check https://de.mathworks.com/help/ros/gs/ros-system-requirements.html for the requirements of the specified MATLAB version.
PYTHON_VERSION=3.9.6
CMAKE_VERSION=3.23.1
GCC_VERSION=10
BOOST_VERSION=1_81_0
BOOST_VERSION2=1.81.0

# To change the Matlab toolboxes to install, change the value of TOOLBOXES.
# Make sure to replace spaces by underscores and seperate toolboxes by spaces.
TOOLBOXES="Parallel_Computing_Toolbox ROS_Toolbox Statistics_and_Machine_Learning_Toolbox"

# To change the network license server address for Matlab, change the value of LICENSE_SERVER.
# The value should be of the format <port>@<host>.
LICENSE_SERVER=50022@license3.rz.rwth-aachen.de

# To change the name or tag of the docker image pushed to the Gitlab container registry, change the value of IMAGE_NAME resp. IMAGE_TAG.
IMAGE_NAME=matlab
IMAGE_TAG=latest

# ---------------------------------------- SCRIPT - DO NOT CHANGE -------------------------------------------

TAG=registry.git-ce.rwth-aachen.de/cpm/coincar/software/graph_based_planning/$IMAGE_NAME:$IMAGE_TAG \

sudo docker build -t $TAG \
  --build-arg MATLAB_RELEASE=$MATLAB_RELEASE \
  --build-arg UBUNTU_VERSION=$UBUNTU_VERSION \
  --build-arg PYTHON_VERSION=$PYTHON_VERSION \
  --build-arg CMAKE_VERSION=$CMAKE_VERSION \
  --build-arg BOOST_VERSION=$BOOST_VERSION \
  --build-arg BOOST_VERSION2=$BOOST_VERSION2 \
  --build-arg GCC_VERSION=$GCC_VERSION \
  --build-arg TOOLBOXES="$TOOLBOXES" \
  --build-arg LICENSE_SERVER=$LICENSE_SERVER .

if $PUSH_IMAGE; then
    sudo docker login registry.git-ce.rwth-aachen.de -u $GITLAB_USERNAME -p $GITLAB_ACCESS_TOKEN
    sudo docker push $TAG
fi
