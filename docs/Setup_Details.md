# Setup Details

- [Features](#features)
- [Installation](#installation)
- [Troubleshooting](#troubleshooting)
- [Requirements Remarks](#requirements-remarks)

# Features

- [C++ Functionalities](#c-functionalities)
- [Lanelet2 Map](#lanelet2-map)

## C++ Functionalities

Some functionalities such as the graph search can optionally done in C++. The written code uses CMake and the C++ Boost libraries. In order to use them the following requirements must be met.

- [Install CMake 3.19+](#cmake)
- [Install C++ Compiler](#c-compiler)
    - Windows: Microsoft Visual Studio Compiler 2019
    - Ubuntu: GCC and G++ Version 10.x+
- [Install Boost 1.76](#c-boost)
- [Install build-essential](#ubuntu-basics) for Ubuntu

## Lanelet2 Map

Under Ubuntu we use lanelet2 as one further map representation, because it offers libraries with lots of useful functionality. Since the libraries only exist in Python and C++, mex functions are used to benefit from that. In order to compile these mex functions the following requirements need to be met.

- [Install Rosless Lanelet2](#rosless-lanelet2)
- Get Git Submodules: `git submodule update --init`

# Installation

- [Windows Path Environment Variable](#windows-path-environment-variable)
- [Ubuntu Basics](#ubuntu-basics)
- [Matlab](#matlab)
- [Python](#python)
- [CMake](#cmake)
- [C++ Compiler](#c-compiler)
- [C++ Boost](#c-boost)
- [Rosless Lanelet2](#rosless-lanelet2)

## Windows Path Environment Variable
Add a path to Path Environment Variable:
1. Search for *Environment Variables*
2. Select option *Environment Variables*
3. *Edit* the variable *Path*
4. Choose *New* for adding the path

## Ubuntu basics
- Package build-essential to compile basic C/C++ software: <br> `sudo apt install build-essential`

## Matlab
- Windows: Installation via GUI
    1. Download [Matlab](https://de.mathworks.com/downloads/)
    2. Run the installer
- Ubuntu: Installation via GUI <br>
    (Since Matlab installation does not follow the unix-style, you could install it in the directory `“/opt/matlab/R2023a”`)
    1. Download [Matlab](https://de.mathworks.com/downloads/)
    2. Extract the zip-file and change to the unzipped directory <br>
    `unzip matlab_R2023a_glnxa64.zip -d matlab` <br>
    `cd matlab`
    3. Run the installer <br>
    `sudo ./install`
    4. Remove files no longer needed <br>
    `cd ..` <br>
    `rm -rf matlab` <br>
    `rm matlab_R2023a_glnxa64.zip`
- Ubuntu: Create symbolic links to have Matlab available in every path <br>
`ln -s /opt/matlab/bin/matlab /usr/local/bin/matlab` <br>
If the link already exist,
    - either [delete it](https://de.mathworks.com/matlabcentral/answers/96149-what-are-symbolic-links-and-why-does-the-matlab-installer-ask-if-i-want-to-create-them): `rm /usr/local/bin/matlab`
    - or [replace it](https://de.mathworks.com/matlabcentral/answers/99535-how-do-i-create-symbolic-links-automatically-using-the-silent-installer): `ln -s --force /opt/matlab/bin/matlab /usr/local/bin/matlab`

## Python
- Windows: Install Python 3.9 via the provided binary ([Windows Download](https://www.python.org/downloads/windows/))
- Ubuntu: Install Python 3.9 via package manager and official ppa
    - `sudo apt install python3.9 python3.9-venv python3.9-dev`
- Ubuntu: Install Python 3.9 via package manager and deadsnakes ppa for newer versions
    - Add an additional package repository: <br>
    `sudo add-apt-repository ppa:deadsnakes/ppa`
    - Update the apt cache: `sudo apt update`
    - Install python: `sudo apt install python3.9 python3.9-venv python3.9-dev`
- Ubuntu: Install Python 3.9.6 via build
    1. Install prerequisites for building Python <br>
       `sudo apt install libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev`
    2. Download python zip-file <br>
       `wget https://www.python.org/ftp/python/3.9.6/Python-3.9.6.tgz`
    3. Extract the zipped file and move in the directory <br>
       `tar -xzf Python-3.9.6.tgz` <br>
       `cd Python-3.9.6`
    4. Execute configure script. It will carry out some tests to ensure that the necessary dependencies are available. (options: python binary is optimized, shared libraries are build, pip is installed, installation directory will be “/usr/local”) <br>
       `./configure --enable-optimizations --enable-shared --with-ensurepip=install --prefix=/usr/local`
    5. Initiate the build process of the python binaries. <br> `make`
    6. Install python at the system. Use “altinstall” to not overwrite the default python version. <br> `sudo make altinstall`
    7. Remove files no longer needed <br>
       `sudo cd ..` <br>
       `rm Python-3.9.6.tgz` <br>
       `rm -rf Python-3.9.6`
    8. EOL: Uninstall python with (https://unix.stackexchange.com/questions/190794/uninstall-python-installed-by-compiling-source) <br>
        `sudo rm -f /usr/local/bin/python3.9` <br>
        `sudo rm -f /usr/local/bin/pip3.9` <br>
        `sudo rm -f /usr/local/bin/pydoc` <br>
        `sudo rm -rf /usr/local/bin/include/python3.9` <br>
        `sudo rm -f /usr/local/lib/libpython3.9.a` <br>
        `sudo rm -rf /usr/local/lib/python3.9`
- Add Python to system path environment variable ([Guide](https://realpython.com/add-python-to-path/))
    - See [Path Environment Variable](#windows-path-environment-variable)
    - Windows: Standard installation path would look like <br>
    `"C:\Users\USER\AppData\Local\Programs\Python\Python39\"`
    - Ubuntu: Not necessary
- Setup Python in Matlab command window as described in the [ROS Toolbox Requirements](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html)
    - Check the Python version of Matlab: `pyenv`
    - Configure a specific Python version: `pyenv(Version="3.9")`
    - If Matlab cannot find Python, try: `pyenv(Version=FullPath/Executable)`

## CMake
- Windows: Install CMake via the provided binary ([Windows Download](<https://cmake.org/download/>))
- Ubuntu: [Install CMake](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line) latest version under Ubuntu via [Kitware's PPA](https://apt.kitware.com/)
    1. Obtain a copy of kitware's signing key <br>
        `wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null`
    2. Add kitware's repository to your sources list and update <br>
        `sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'` <br> `sudo apt update`
    3. Install the kitware-archive-keyring package to ensure that your keyring stays up to date as we rotate our keys <br>
        `sudo apt install kitware-archive-keyring` <br> `sudo rm /etc/apt/trusted.gpg.d/kitware.gpg`
    4. Note: If running sudo apt update gets the following error

        > Err:7 https://apt.kitware.com/ubuntu bionic InRelease <br>
        The following signatures couldn't be verified because the public key is not available: NO_PUBKEY 6AF7F09730B3F0A4 <br>
        Fetched 11.0 kB in 1s (7552 B/s)

        Copy the public key *6AF7F09730B3F0A4* and run the command <br>
        `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 6AF7F09730B3F0A4`
    5. Finally we can update and install the cmake package <br>
        `sudo apt update` <br> `sudo apt install cmake`
- Add CMake to system path environment variable
    - See [Path Environment Variable](#windows-path-environment-variable)
    - Windows: Standard installation path would look like <br>
    `"C:\Program Files\CMake\bin\"`
    - Ubuntu: Not necessary

## C++ Compiler
- Windows: Install the [Visual Studio 2019 Community Edition](https://learn.microsoft.com/en-us/visualstudio/releases/2019/release-notes). The (free) Community Edition is sufficient. Select "Desktop development with C++" during the installation.
- Ubuntu: Install gcc and g++ officially supported versions
    - Install gcc/g++: `sudo apt install gcc-7 g++-7`
    - Set new gcc/g++ versions as default: <br> `sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70 --slave /usr/bin/g++ g++ /usr/bin/g++-7`
- Ubuntu: [Install gcc and g++](https://linuxize.com/post/how-to-install-gcc-compiler-on-ubuntu-18-04/) newer versions
    - Add an additional package repository `sudo add-apt-repository ppa:ubuntu-toolchain-r/test`
    - Update the apt cache: `sudo apt update`
    - Install gcc/g++: `sudo apt install gcc-10 g++-10`
    - Set new gcc/g++ versions as default: <br> `sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ g++ /usr/bin/g++-10`
- Setup the compiler in the Matlab command windows: `mex -setup`

## C++ Boost
- Boost contains mostly header-only libraries (no build required) but also some libraries that must be built. These libraries are built with Boost.Build (B2)
- Windows: [Installation Guide](https://www.boost.org/doc/libs/1_76_0/more/getting_started/windows.html)
    1. Download [boost_1_76_0.7z](https://www.boost.org/users/history/version_1_76_0.html)
    2. Extract the zipped file into `"C:\Program Files\boost\boost_1_76_0"` <br>
    (This directory is sometimes referred to as $BOOST_ROOT)
    3. In Visual Studio: Add the directory to C/C++ *Additional Include Directories*
    4. In the boost directory run `bootstrap.bat` to prepare Boost.Build
    5. In the boost directory run `.\b2.exe` to build the libraries (this takes some time)
    6. [CMake FindBoost](https://cmake.org/cmake/help/latest/module/FindBoost.html) searches for Boost in [Environment Variables that must be set](https://github.com/giotto-ai/giotto-tda/issues/115#issuecomment-571502945) <br>
    *BOOST_ROOT* = `"C:\Program Files\boost\boost_1_76_0"` <br>
    *BOOST_INCLUDEDIR* = `"C:\Program Files\boost\boost_1_76_0"` <br> *BOOST_LIBRARYDIR* = `"C:\Program Files\boost\boost_1_76_0\stage\lib"`
- Ubuntu: [Installation Guide](https://www.boost.org/doc/libs/1_76_0/more/getting_started/unix-variants.html) or [External Installation Guide](https://linux.how2shout.com/how-to-install-boost-c-on-ubuntu-20-04-or-22-04/)
    1. Install dependencies: `sudo apt-get install build-essential g++ python-dev autotools-dev libicu-dev libbz2-dev libboost-all-dev`
    2. Download [boost_1_76_0.tar.bz2](https://www.boost.org/users/history/version_1_76_0.html): `wget https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.bz2`
    3. Extract it: `tar -xvjf boost_1_76_0.tar.bz2` and change directory: `cd boost_1_76_0`
    4. Prepare installation: `./bootstrap.sh --prefix=/usr/local`. If there are multiple boost versions on your system, you might want to install boost in a local folder like `install` instead of `/usr/local`.
    5. Install libraries (header only and build required): `sudo ./b2 install`
    6. Remove files no longer needed: <br>
    `sudo cd ..` <br>
    `rm boost_1_76_0.tar.bz2` <br>
    `rm -rf boost_1_76_0`
    7. EOL: Uninstall boost with: (https://stackoverflow.com/questions/8430332/uninstall-boost-and-install-another-version) <br>
    `sudo apt-get -y --purge remove libboost-all-dev libboost-doc libboost-dev` <br>
    `echo "clear boost dir"` <br>
    `sudo rm -r /usr/local/lib/libboost*` <br>
    `sudo rm -r /usr/local/include/boost` <br>
    `sudo rm -r /usr/local/lib/cmake/*` <br>
    `sudo rm -f /usr/lib/libboost_*` <br>
    `sudo rm -r /usr/include/boost`

## Rosless Lanelet2
- Dependencies:
    - boost
    - eigen3
    - pugixml (for lanelet2_io)
    - geographic (for lanelet2_projection)
- Preparation:
    1. [Install C++ boost version 1.76](#c-boost)
    2. Install further dependencies of Lanelet2 </br>
    `sudo apt install libboost-dev libeigen3-dev libgeographic-dev libpugixml-dev`
    3. Clone the repository [Rosless-Lanelet2 from Github](https://github.com/embedded-software-laboratory/Rosless-Lanelet2) </br>
    `git clone https://github.com/embedded-software-laboratory/Rosless-Lanelet2.git`
    4. Change directory to the cloned repository </br> `cd Rosless-Lanelet2`
    5. Checkout specific commit to ensure the desired behavior </br>
    `git checkout 0f190ed17d5060bc30eb03d7ac0d10bf06702096`
- Installation:
    1. Make build directory and change to it </br> `mkdir build` </br> `cd build`
    2. Configure the project and specify the installation directory </br> (default installation destinations are in '/usr/local') </br>
    `cmake .. -DCMAKE_INSTALL_PREFIX=<install-dir>`. If you installed the needed boost version in a local folder like `install` as mentioned above (step 4 of Ubuntu Boost Installation), you furthermore need to specify this folder here by using `-DBOOST_ROOT=<path-to-your-boost-install-folder>`
    3. Build the target install to install the libraries </br>
    (sudo rights are required for write access at the destinations) </br>
    `sudo cmake --build . --target install`
    4. Restart the computer that the environment variables can be updated

# Troubleshooting

- [Matlab](#matlab-1)
- [Rosless Lanelet2](#rosless-lanelet2-1)

## Matlab

Installing Matlab Updates or Toolboxes afterwards causes permission problems
- > Permission denied
- Since Matlab was installed with sudo, sudo rights are required to install a toolbox. But since Matlab is always started without sudo, the toolbox installer is started without sudo as well.
- Changing the owner of the Matlab installation directory solves this problem:
<br> `sudo chown -R $LOGNAME: /usr/opt/MATLAB/R2023a`
<br> (https://de.mathworks.com/matlabcentral/answers/334889-can-t-install-any-toolboxes-because-can-t-write-to-usr-local-matlab-r2017, https://de.mathworks.com/matlabcentral/answers/315712-why-do-i-receive-access-denied-or-folder-error-when-installing-matlab-on-linux)

Starting Matlab from terminal a message appears
- > Gtk-Message: <timestamp>: Failed to load module "canberra-gtk-module"
- Message is informal and has no effects on Matlab (https://de.mathworks.com/support/bugreports/1995075, https://de.mathworks.com/matlabcentral/answers/543536-matlab-on-linux-failed-to-load-module-canberra-gtk-module)

On startup Matlab command window shows a warning
- > Warning: X does not support locale en_US.UTF-8
- The warning message that can be disregarded and it has no impact on the functionality of MATLAB (https://www.mathworks.com/matlabcentral/answers/1929870-why-do-i-get-warning-on-startup-x-does-not-support-locale-en_us-utf-8-when-startup-matlab-with-r2?s_tid=answers_rc1-2_p2_MLT)

Searching for cmake via Matlab command window returns an error
- `!cmake --version` or `system('cmake --version')`
- ... results in error:
- > cmake: /usr/local/MATLAB/R2023a/bin/glnxa64/libcurl.so.4: no version information available (required by cmake) <br>
cmake: /usr/local/MATLAB/R2023a/sys/os/glnxa64/libstdc++.so.6: version 'GLIBCXX_3.4.30' not found (required by cmake) <br>
cmake: /usr/local/MATLAB/R2023a/sys/os/glnxa64/libstdc++.so.6: version 'GLIBCXX_3.4.29' not found (required by cmake) <br>
cmake: /usr/local/MATLAB/R2023a/sys/os/glnxa64/libstdc++.so.6: version 'GLIBCXX_3.4.29' not found (required by /lib/x86_64-linux-gnu/libjsoncpp.so.25)
- Similar errors: (https://de.mathworks.com/matlabcentral/answers/1461849-why-does-running-the-unix-curl-command-from-within-matlab-result-in-an-error, https://de.mathworks.com/matlabcentral/answers/1680154-matlab-cannot-find-cmake-on-ubuntu-20-04, https://de.mathworks.com/matlabcentral/answers/329796-issue-with-libstdc-so-6)
- Explanation: The libraries shipped with Matlab are incompatible with the operating system
- Force Matlab to load the external libcurl.so.4 with environment variable $LD_LIBRARY_PATH <br>
(The path of the external libcurl.so.4 can be found out with `whereis libcurl.so.4`) <br>
(https://de.mathworks.com/matlabcentral/answers/1461849-why-does-running-the-unix-curl-command-from-within-matlab-result-in-an-error)
- Check if version of the external libstdc++.so.6 is available with <br>
`strings 'PATH'/libstdc++.so.6 | grep GLIBCXX` <br>
(The path of the external libstdc++.so.6 can be found out with `whereis libstdc++.so.6`) <br>
(https://de.mathworks.com/matlabcentral/answers/329796-issue-with-libstdc-so-6)
- If sufficient, update libstdc++6 <br>
`sudo apt install libstdc++6` <br>
(https://askubuntu.com/questions/726539/sudo-apt-get-install-libstdc)
- If necessary, upgrade libstdc++6 via package repository ppa:ubuntu-toolchain-r/test <br>
`sudo add-apt-repository ppa:ubuntu-toolchain-r/test` <br>
`sudo apt-get update` <br>
`sudo apt upgrade libstdc++6` <br>
(https://github.com/LedgerHQ/ledger-live-desktop/issues/4016#issuecomment-889455692)
- Force Matlab to load the external libstdc++.so.6 with environment variable $LD_PRELOAD <br>
(https://de.mathworks.com/matlabcentral/answers/329796-issue-with-libstdc-so-6)

Windows: ROS Toolbox cannot find CMake (if you want to use an external CMake)
- > Error using ros.internal.utilities.getCMakeBinaryPath
Unable to find CMake in your system. Please install CMake version 3.15.5 or higher and rerun the command.
- Explanation: Starting with Matlab R2022b, manual installation of CMake is not required since it is part of Matlab installation. If this is not the case and CMake is not contained, this leads to an error. The reason for this is that ROS Toolbox searches for the CMake executable in <br> `"C:\Program Files\MATLAB\<VERSION>\bin\win64\cmake\bin\cmake"`. <br> Although, there is a fallback solution, that does not work on Windows OS. <br> `[status, result] = system('which cmake');`
- Possible Solutions:
    1. Installation of the Matlab Toolbox "Matlab Coder" will install CMake in the Matlab installation folder as [suggested by the Matlab Staff](https://de.mathworks.com/matlabcentral/answers/1973764-matlab-r2022b-and-r2023a-installation-did-not-contain-cmake)
    2. Creating a symbolic link for the missing folder *cmake* with the [mklink](https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/mklink?source=recommendations) command in Windows command prompt `mklink /D "C:\Program Files\MATLAB\R2023a\bin\win64\cmake" "C:\Program Files\CMake"`
    3. Adjusting the internal ROS Toolbox script that searches for the CMake executable by substituting `which` with the Windows equivalent `where` in `"C:\Program Files\MATLAB\<VERSION>\toolbox\ros\utilities\+ros\+internal\+utilities\getCMakeBinaryPath.m"`
    4. Create an alias that calls the Windows equivalent `where` instead of `which`. <br>
This can be done with the [doskey](https://blog.doubleslash.de/effizienter-arbeiten-in-der-windows-command-prompt-mit-doskey) command `doskey which = where $*` in Windows command prompt (cmd). Since the alias would be temporary for the active cmd, it must be made permanent which requires adjusting the Windows registry as described [here](https://superuser.com/questions/1134368/create-permanent-doskey-in-windows-cmd) or [here](https://blog.doubleslash.de/effizienter-arbeiten-in-der-windows-command-prompt-mit-doskey).

## Rosless Lanelet2

Boost warning massage during the build process
- > #pragma message: This header is deprecated. Use \<iterator> instead.
- The message occurs for boost versions 1.74, 1.75, and 1.76 since the graph library uses the headers (https://github.com/boostorg/graph/issues/228)

Mex file cannot find `liblanelet2_<>.so`
- Although rosless lanelet2 was installed under `/usr/local/lib` on some computers MATLAB does not find it. Before you then start matlab (from a terminal) run `export LD_LIBRARY_PATH=/usr/local/lib`. The same problem may arise with the boost library when installing in a local folder, e.g., `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path-to-your-boost-install-folder>/lib`.

# Requirements Remarks
Mathworks [Platform Road Map](https://de.mathworks.com/support/requirements/platform-road-map.html) shows an overview of supported OS for Matlab and Simulink.

Matlab Dependency Analyzer found additional toolboxes, that are not used anymore:
- Optimization Toolbox
- Symbolic Math Toolbox

CMake Minimum Version:
- Version 3.16.3+ for Matlab R2022a and earlier (see [ROS Toolbox Requirements](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html))
- Version 3.19.7 shipped with Matlab R2022b and later (Windows command prompt: <br>`"C:\Program Files\MATLAB\'VERSION'\bin\win64\cmake\bin\cmake.exe" --version`)
- Version 3.19 for graph search (see `.\hlc\optimizer\graph_search_mex\CMakeList.txt`)

GCC Minimum Version:
- Version 6.3+ (see [ROS Toolbox Requirements](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html))
- Version 7.x+ (see [Supported Compilers](https://de.mathworks.com/support/requirements/supported-compilers-linux.html))
- Version 10.x+ (see [C++20 Feature "Coroutines" Requirements](https://gcc.gnu.org/projects/cxx-status.html#cxx20))

Boost Version:
- Version 1.76 (latest version which we managed running with Lanelet2)