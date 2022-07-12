# Receding Horizon Graph Search
<!-- icons from https://simpleicons.org/ -->
<!-- [![Paper](https://img.shields.io/badge/-Paper-00629B?logo=IEEE)]()  -->
[![Repository](https://img.shields.io/badge/-GitHub-181717?logo=GitHub)](https://github.com/embedded-software-laboratory/receding-horizon-graph-search) 
[![Video](https://img.shields.io/badge/-Video-FF0000?logo=YouTube)](https://www.youtube.com/watch?v=7LB7I5SOpQE) 
[![Open in Code Ocean](https://codeocean.com/codeocean-assets/badge/open-in-code-ocean.svg)](https://codeocean.com/capsule/7778016/tree/v2)

<!-- GIF: ffmpeg -y -i video_3-circle_RHC.mp4 -vf "crop=in_h:in_h:420:0,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 preview.gif -->
<!-- https://gifyu.com/image/GGVg -->
<img src="https://s9.gifyu.com/images/preview6ec7fc3f8852dd89.gif" alt="preview6ec7fc3f8852dd89.gif" width="500"/>

This repository contains the source code for Receding Horizon Graph Search (RHGS), a MATLAB implementation of a graph-based receding horizon trajectory planner.

The code is developed with MATLAB R2021a.
To run a simulation:
```matlab
startup()
main()
```

More information is provided in our publication [1], which we kindly ask you to consider citing if you find RHGS helpful for your work.
The results of the publication can be reproduced by running
```matlab
startup()
eval_paper()
```
This will take a while. The results are then found in the folder "results".
### Configure Your MATLAB for ROS 2
Vehicles communicate with each other using MATLAB ROS 2 toolbox. Message types are customized, which requires to run the MATLAB build-in function `ros2genmsg()`. Note this needs to be done only once for one machine. There are some general steps to help you configure your MATLAB so that `ros2genmsg()` can run successfully. In total there are three things needed to be configured: Python, CMake and Microsoft Visual Studio. Note that you can check which version is needed in the MATLAB official doc: https://de.mathworks.com/help/ros/gs/ros-system-requirements.html.
- Window
1. Ensure your machine has Python with the right version installed. Please check the above line to see which Python version you need. For example, Python 3.9 is needed for MATLAB R2022a. After you have installed Python with the right version, you can go to MATLAB command window and use `pyenv('Version','version')` to set up MATLAB. For example, `pyenv('Version','3.9')` will let MATLAB use Python 3.9. Make sure you have added its path to you environment (many instructions online available, such as https://geek-university.com/add-Python-to-the-windows-path/); otherwise, you should use something like `pyenv('Version','fullPathOfYourPythonInstallFolder\Python.exe')`.
2. Ensure your machine has CMake with the right version installed. You can check this using `!cmake --version` in MATLAB command window. For MATLAB R2022a, CMake 3.16.3+ is needed. You can install it at https://cmake.org/download/. Note that add path to environment is recommended.
3. Ensure you machine has Visual Studio with the right version installed and set for MATLAB. You can check and set this using `mex -setup` in MATLAB command window. For MATLAB R2022a, Visual Studio 2019 is needed. You can download it at https://docs.microsoft.com/en-us/visualstudio/releases/2019/release-notes. Note that the (free) Community of Version Studio is already enough. When installing and `Desktop development with C++` must be selected when installing. 
- Linux (TODO)

### Acknowledgements
This research is supported by the Deutsche Forschungsgemeinschaft (German Research Foundation) within the Priority Program SPP 1835 "Cooperative Interacting Automobiles" (grant number: KO 1430/17-1).

### References

<details>
<summary>
[1] P. Scheffe, M. V. A. Pedrosa, K. Flaßkamp and B. Alrifaee,
"Receding Horizon Control Using Graph Search for Multi-Agent Trajectory Planning"
</summary>
<p>
```
bibtex
```
</p>
</details>
