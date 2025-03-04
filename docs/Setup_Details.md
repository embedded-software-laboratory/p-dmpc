# Setup Details

- [Setup Details](#setup-details)
- [Installation](#installation)
  - [Windows Path Environment Variable](#windows-path-environment-variable)
  - [Ubuntu basics](#ubuntu-basics)
  - [Matlab](#matlab)
  - [Python](#python)
  - [C++ Compiler](#c-compiler)
- [Troubleshooting](#troubleshooting)
  - [Matlab](#matlab-1)
- [Requirements Remarks](#requirements-remarks)

# Installation

## Windows Path Environment Variable

Add a path to Path Environment Variable:

1. Search for _Environment Variables_
2. Select option _Environment Variables_
3. _Edit_ the variable _Path_
4. Choose _New_ for adding the path

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
  If the link already exist, - either [delete it](https://de.mathworks.com/matlabcentral/answers/96149-what-are-symbolic-links-and-why-does-the-matlab-installer-ask-if-i-want-to-create-them): `rm /usr/local/bin/matlab` - or [replace it](https://de.mathworks.com/matlabcentral/answers/99535-how-do-i-create-symbolic-links-automatically-using-the-silent-installer): `ln -s --force /opt/matlab/bin/matlab /usr/local/bin/matlab`

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

# Troubleshooting

- [Matlab](#matlab-1)

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
  > cmake: /usr/local/MATLAB/R2023a/sys/os/glnxa64/libstdc++.so.6: version 'GLIBCXX_3.4.30' not found (required by cmake) <br>
  > cmake: /usr/local/MATLAB/R2023a/sys/os/glnxa64/libstdc++.so.6: version 'GLIBCXX_3.4.29' not found (required by cmake) <br>
  > cmake: /usr/local/MATLAB/R2023a/sys/os/glnxa64/libstdc++.so.6: version 'GLIBCXX_3.4.29' not found (required by /lib/x86_64-linux-gnu/libjsoncpp.so.25)
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
  > Unable to find CMake in your system. Please install CMake version 3.15.5 or higher and rerun the command.
- Explanation: Starting with Matlab R2022b, manual installation of CMake is not required since it is part of Matlab installation. If this is not the case and CMake is not contained, this leads to an error. The reason for this is that ROS Toolbox searches for the CMake executable in <br> `"C:\Program Files\MATLAB\<VERSION>\bin\win64\cmake\bin\cmake"`. <br> Although, there is a fallback solution, that does not work on Windows OS. <br> `[status, result] = system('which cmake');`
- Possible Solutions: 1. Installation of the Matlab Toolbox "Matlab Coder" will install CMake in the Matlab installation folder as [suggested by the Matlab Staff](https://de.mathworks.com/matlabcentral/answers/1973764-matlab-r2022b-and-r2023a-installation-did-not-contain-cmake) 2. Creating a symbolic link for the missing folder _cmake_ with the [mklink](https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/mklink?source=recommendations) command in Windows command prompt `mklink /D "C:\Program Files\MATLAB\R2023a\bin\win64\cmake" "C:\Program Files\CMake"` 3. Adjusting the internal ROS Toolbox script that searches for the CMake executable by substituting `which` with the Windows equivalent `where` in `"C:\Program Files\MATLAB\<VERSION>\toolbox\ros\utilities\+ros\+internal\+utilities\getCMakeBinaryPath.m"` 4. Create an alias that calls the Windows equivalent `where` instead of `which`. <br>
  This can be done with the [doskey](https://blog.doubleslash.de/effizienter-arbeiten-in-der-windows-command-prompt-mit-doskey) command `doskey which = where $*` in Windows command prompt (cmd). Since the alias would be temporary for the active cmd, it must be made permanent which requires adjusting the Windows registry as described [here](https://superuser.com/questions/1134368/create-permanent-doskey-in-windows-cmd) or [here](https://blog.doubleslash.de/effizienter-arbeiten-in-der-windows-command-prompt-mit-doskey).

# Requirements Remarks

Mathworks [Platform Road Map](https://de.mathworks.com/support/requirements/platform-road-map.html) shows an overview of supported OS for Matlab and Simulink.

Matlab Dependency Analyzer found additional toolboxes, that are not used anymore:

- Optimization Toolbox
- Symbolic Math Toolbox

CMake Minimum Version:

- Version 3.16.3+ for Matlab R2022a and earlier (see [ROS Toolbox Requirements](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html))
- Version 3.19.7 shipped with Matlab R2022b and later (Windows command prompt: <br>`"C:\Program Files\MATLAB\'VERSION'\bin\win64\cmake\bin\cmake.exe" --version`)

GCC Minimum Version:

- Version 6.3+ (see [ROS Toolbox Requirements](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html))
- Version 7.x+ (see [Supported Compilers](https://de.mathworks.com/support/requirements/supported-compilers-linux.html))

Boost Version:

- Version 1.76
