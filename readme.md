# Priority-Based Trajectory Planning for Networked Vehicles Using Motion Primitives
- [Priority-Based Trajectory Planning for Networked Vehicles Using Motion Primitives](#priority-based-trajectory-planning-for-networked-vehicles-using-motion-primitives)
- [Setup](#setup)
    - [MATLAB R2023a](#matlab-r2023a)
    - [System Requirements for MATLAB ROS 2 Toolbox](#system-requirements-for-matlab-ros-2-toolbox)
    - [System Requirements for using Lanelet2](#system-requirements-for-using-lanelet2)
- [Run Exeriments](#run-exeriments)
- [References](#references)
- [Acknowledgements](#acknowledgements)
# Setup
## MATLAB R2023a
Install MATLAB R2023a with the following toolboxes:
- Statistics and Machine Learning Toolbox
- ROS Toolbox
- Parallel Computing Toolbox
## System Requirements for MATLAB ROS 2 Toolbox
In our priority-based trajectory planning, vehicles communicate using the MATLAB ROS 2 toolbox. Their custom messages are compiled with the MATLAB built-in function `ros2genmsg()`, for which you must have Python software, CMake software, and a C++ compiler for your platform ([ROS Toolbox Requirements](https://de.mathworks.com/help/ros/gs/ros-system-requirements.html)).
For MATLAB R2023a
1. Python 3.9
    1. Install Python 3.9 and add it to your system path variable.
    2. Go to the MATLAB command window and execute `pyenv('Version','version')` to set up Python with MATLAB. For example, `pyenv('Version','3.9')` will let MATLAB use Python 3.9. If MATLAB cannot find the version, provide its path with `pyenv('Version','fullPathOfYourPythonInstallFolder\YourPythonExecutable')`.
2. CMake 3.16.3+
    1. Install from https://cmake.org/download/
3. C++ compiler
    1. Installation
        * Windows: Install the [Visual Studio 2019 Community Edition](https://learn.microsoft.com/en-us/visualstudio/releases/2019/release-notes). The (free) community version is sufficient. Select "Desktop development with C++" during the installation.
    2. Configure your C++ compiler using `mex -setup` in the MATLAB command window.
## System Requirements for using Lanelet2
As one map representation we use lanelet2, because it offers libraries with lots of useful functionality. Since the libraries only exist in Python and C++, mex functions are used to benefit from that. In order to compile these mex functions the following dependencies need to be met.
1. Lanelet2
    1. Install according to https://github.com/fzi-forschungszentrum-informatik/Lanelet2#installation . Under Ubuntu22 you probably want to use it as part of the ROS2 humble version, i.e., `sudo apt install ros-humble-lanelet2`
2. Eigen3
    1. Install Eigen3 as dependency of Lanelet2. In Ubuntu, the following command should be sufficient: `sudo apt install libeigen3-dev`
3. Git Submodules
    1. Since some of our functionality is added by using git submodules, use `git submodule init` after cloning to retrieve the content of these repos.

# Run Exeriments

See [here](/doc/Run_Experiments.md)

# References
Please refer to the respective publication if you are using it for your work. Thank you very much!

<details>
<summary>
P. Scheffe, J. Xu and B. Alrifaee, "Limiting Computation Levels in Prioritized Trajectory Planning with Safety Guarantees", ResearchGate, Preprint, 2023, doi: 10.13140/RG.2.2.32731.03368
<br>

<!-- icons from https://simpleicons.org/ -->
[![Paper](https://img.shields.io/badge/Preprint-Paper-00629B)](http://dx.doi.org/10.13140/RG.2.2.32731.03368)
[![Repository](https://img.shields.io/badge/-GitHub-181717?logo=GitHub)](https://github.com/embedded-software-laboratory/p-dmpc)
[![Video](https://img.shields.io/badge/-Video-FF0000?logo=YouTube)](https://youtu.be/alGHLwQQpHI)
</summary>
<p>

The results of the publication can be reproduced by running
```matlab
open graph_based_planning.prj
eval_parallel_computation_prediction_inconsistency()
eval_parallel_computation_CLs()
```
The results are saved in the folder "results".

</p>
</details>

<details>
<summary>
P. Scheffe, J. Kahle and B. Alrifaee, "Reducing Computation Time with Priority Assignment in Distributed MPC," TechRxiv, Preprint, 2023, doi: 10.36227/techrxiv.20304015.v2
<br>

<!-- icons from https://simpleicons.org/ -->
[![Paper](https://img.shields.io/badge/Preprint-Paper-00629B)](https://doi.org/10.36227/techrxiv.20304015.v2)
[![Repository](https://img.shields.io/badge/-GitHub-181717?logo=GitHub)](https://github.com/embedded-software-laboratory/p-dmpc)
</summary>
<p>

The results of the publication can be reproduced by running
```matlab
eval_coloring_paper()
```
This evaluation comprises 720 simulations, so it will take days until completion.
The results are saved in the folder "results".

</p>
</details>

<details>
<summary>
P. Scheffe and B. Alrifaee, "A Scaled Experiment Platform to Study Interactions Between Humans and CAVs", ResearchGate, Preprint, 2023, doi: 10.13140/RG.2.2.24697.13923
<br>

<!-- icons from https://simpleicons.org/ -->
[![Paper](https://img.shields.io/badge/Preprint-Paper-00629B)](https://dx.doi.org/10.13140/RG.2.2.24697.13923)
[![Repository](https://img.shields.io/badge/-GitHub-181717?logo=GitHub)](https://github.com/embedded-software-laboratory/p-dmpc)
[![Video](https://img.shields.io/badge/-Video-FF0000?logo=YouTube)](https://youtu.be/G93nqfdmD48)
</summary>
<p>

The results of the publication can be reproduced by running
```matlab
hdv_reachable_set_experiment()
```
The results are saved in the folder "results".

</p>
</details>

<details>
<summary>
P. Scheffe, M. V. A. Pedrosa, K. Flaßkamp and B. Alrifaee, "Receding Horizon Control Using Graph Search for Multi-Agent Trajectory Planning", in IEEE Transactions on Control Systems Technology, 2022, doi: 10.1109/TCST.2022.3214718.

<!-- icons from https://simpleicons.org/ -->
[![Paper](https://img.shields.io/badge/-Paper-00629B?logo=IEEE)](https://doi.org/10.1109/TCST.2022.3214718)
[![Repository](https://img.shields.io/badge/-GitHub-181717?logo=GitHub)](https://github.com/embedded-software-laboratory/p-dmpc/tree/pub/rhgs)
[![Video](https://img.shields.io/badge/-Video-FF0000?logo=YouTube)](https://www.youtube.com/watch?v=7LB7I5SOpQE)
[![Code Ocean](https://codeocean.com/codeocean-assets/badge/open-in-code-ocean.svg)](https://codeocean.com/capsule/7778016)
</summary>
<p>
<img src="./docs/media/3-circle_rhgs.gif" width=640/>

The results of the publication can be reproduced by running
```matlab
eval_rhgs()
```
The results are saved in the folder "results".

</p>
</details>

<details>
<summary>
References in Bibtex format
</summary>
<p>

```bibtex
@article{scheffe2023reducing,
    author = {Patrick Scheffe and Julius Kahle and Bassam Alrifaee},
    title  = {Reducing Computation Time with Priority Assignment in Distributed MPC},
    year   = {2023},
    month  = {2},
    doi    = {10.36227/techrxiv.20304015.v2}
}

@article{scheffe2023scaled,
    author = {Patrick Scheffe and Bassam Alrifaee},
    title  = {A Scaled Experiment Platform to Study Interactions Between Humans and CAVs},
    year   = {2023},
    month  = {2},
    doi    = {10.13140/RG.2.2.24697.13923}
}

@article{scheffe2022receding,
    author  = {Patrick Scheffe and Matheus Vitor de Andrade Pedrosa and Kathrin Flaßkamp and Bassam Alrifaee},
    journal = {IEEE Transactions on Control Systems Technology},
    title   = {Receding Horizon Control Using Graph Search for Multi-Agent Trajectory Planning},
    year    = {2022},
    volume  = {},
    number  = {},
    pages   = {1-14},
    doi     = {10.1109/TCST.2022.3214718}
}

@article{scheffe2023prediction,
    author  = {Patrick Scheffe and Jianye Xu and Bassam Alrifaee},
    journal = {todo},
    title   = {A Prediction Consistent Framework to Limit Computation Time in Priority-Based Trajectory Planning},
    year    = {2023},
    volume  = {},
    number  = {},
    pages   = {},
    doi     = {todo}}
}
```

</p>
</details>







# Acknowledgements
This research is supported by the Deutsche Forschungsgemeinschaft (German Research Foundation) within the Priority Program SPP 1835 "Cooperative Interacting Automobiles" (grant number: KO 1430/17-1).
