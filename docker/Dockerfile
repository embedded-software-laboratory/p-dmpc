### This file is inspired by https://github.com/mathworks-ref-arch/matlab-dockerfile.


### Use Ubuntu image of specified version as base image.
ARG UBUNTU_VERSION=18.04
FROM ubuntu:$UBUNTU_VERSION


### Install basic dependencies.
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install --no-install-recommends --yes \
        build-essential \
        ca-certificates \
        git \
        libxt6 \
        libxext6 \
        software-properties-common \
        unzip \
        wget


### Install MATLAB and ROS2 Toolbox dependencies as listed in https://de.mathworks.com/help/ros/gs/ros-system-requirements.html.

# Install specified Python version, defaults to python 3.9.6.
ARG PYTHON_VERSION=3.9.6
RUN apt-get install --no-install-recommends --yes libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev
RUN wget -q https://www.python.org/ftp/python/$PYTHON_VERSION/Python-$PYTHON_VERSION.tgz \
    && tar xzf Python-$PYTHON_VERSION.tgz \
    && cd Python-$PYTHON_VERSION \
    && ./configure --enable-optimizations --with-ensurepip=install --enable-shared --prefix=/usr \
    && make \
    && make altinstall \
    && cd .. && rm Python-$PYTHON_VERSION.tgz && rm -rf Python-$PYTHON_VERSION

# Install specified gcc version, defaults to gcc 6, and set installed version as default.
ARG GCC_VERSION=10
RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get install --no-install-recommends --yes gcc-$GCC_VERSION g++-$GCC_VERSION
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-$GCC_VERSION 60 --slave /usr/bin/g++ g++ /usr/bin/g++-$GCC_VERSION

# Update libstdc++6 to newer version to be compatible with Matlab as described in
# https://github.com/LedgerHQ/ledger-live-desktop/issues/4016#issuecomment-889455692.
RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get --yes upgrade libstdc++6
# Use installed new libstdc++ version instead of Matlab version as described in
# https://de.mathworks.com/matlabcentral/answers/329796-issue-with-libstdc-so-6#answer_259460.
ENV LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6"

# Clean up.
RUN apt-get clean \
    && apt-get autoremove --yes \
    && rm -rf /var/lib/apt/lists/*


### Run mpm to install MATLAB in the target location.
# Get mpm.
RUN wget -q https://www.mathworks.com/mpm/glnxa64/mpm
RUN chmod +x mpm

# Install matlab and specified toolboxes.
ARG MATLAB_RELEASE=r2023a
ARG TOOLBOXES="Parallel_Computing_Toolbox ROS_Toolbox Statistics_and_Machine_Learning_Toolbox"
RUN ./mpm install --release=$MATLAB_RELEASE --destination=/opt/matlab --products MATLAB $TOOLBOXES

# Make matlab available everywhere with a symbolic link.
RUN ln -s /opt/matlab/bin/matlab /usr/local/bin/matlab

# Set path to libGL.so.1.
ENV LD_PRELOAD="$LD_PRELOAD /opt/matlab/sys/opengl/lib/glnxa64/libGL.so.1"

# Remove mpm installation again.
RUN rm -f mpm /tmp/mathworks_root.log

# Specify the host and port of the machine that serves the network licenses as an environment variable.
ARG LICENSE_SERVER=50022@license3.rz.rwth-aachen.de
ENV MLM_LICENSE_FILE=$LICENSE_SERVER

# Set specified python version as default python in Matlab.
RUN matlab -nodisplay -nosplash -nodesktop -r "pyenv('Version', '/usr/bin/python${PYTHON_VERSION%.*}'); disp(pyenv); exit;"

# Install git-lfs for expected test results
RUN apt-get update && apt-get install git-lfs

### Set working directory.
WORKDIR /home/matlab
CMD [""]
