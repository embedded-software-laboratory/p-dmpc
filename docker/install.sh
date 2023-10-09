#!/bin/bash

# All settings should be possible to set in this SETTINGS sections. The SCRIPT section shouldn't have to be changed.

# --------------------------------------------- UPLOAD SETTINGS ---------------------------------------------------

# Define whether the built image should be pushed to the Gitlab container registry by setting PUSH_IMAGE to true or false. Default is false.
# If set to true, the environment variables GITLAB_USERNAME and GITLAB_ACCESS_TOKEN in the file login_credentials have to be set.
# If you first only build the image and then decide to push, just rerun the script with PUSH_IMAGE=true and the image does not have to be rebuild due to docker's cache.
PUSH_IMAGE=false

# If push_image==true, follow the instructions in the login_credentials.sh file.
if $PUSH_IMAGE; then
  source login_credentials.sh
fi

# --------------------------------------------- IMAGE SETTINGS ----------------------------------------------------
# To install another MATLAB release than R2022a in the container, change the value of MATLAB_RELEASE. Use only lower case.
MATLAB_RELEASE=r2023a

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
BOOST_VERSION=1_76_0
BOOST_VERSION2=1.76.0

# To have a fixed version of Lanelet2 installed on this system, it is needed to specify the commit of the Rosless-Lanelet2 version which shall be used.
# The Rosless-Lanelet2 repository can be found here: https://github.com/embedded-software-laboratory/Rosless-Lanelet2
LANELET2_COMMIT=0f190ed17d5060bc30eb03d7ac0d10bf06702096

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

sudo docker buildx create --use --name larger_log --driver-opt env.BUILDKIT_STEP_LOG_MAX_SIZE=50000000
sudo docker buildx build --load -t $TAG \
  --progress plain \
  --build-arg MATLAB_RELEASE=$MATLAB_RELEASE \
  --build-arg UBUNTU_VERSION=$UBUNTU_VERSION \
  --build-arg PYTHON_VERSION=$PYTHON_VERSION \
  --build-arg CMAKE_VERSION=$CMAKE_VERSION \
  --build-arg BOOST_VERSION=$BOOST_VERSION \
  --build-arg BOOST_VERSION2=$BOOST_VERSION2 \
  --build-arg LANELET2_COMMIT=$LANELET2_COMMIT \
  --build-arg GCC_VERSION=$GCC_VERSION \
  --build-arg TOOLBOXES="$TOOLBOXES" \
  --build-arg LICENSE_SERVER=$LICENSE_SERVER .

if $PUSH_IMAGE; then
    sudo docker login registry.git-ce.rwth-aachen.de -u $GITLAB_USERNAME -p $GITLAB_ACCESS_TOKEN
    sudo docker push $TAG
fi
