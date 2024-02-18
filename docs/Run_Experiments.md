# Sim Lab

- execute main() and select Config in [UI](#UI) or execute main(config).
- If you are using Distributed Execution with Sim Lab options, the Matlab Parallel Computing Toolbox will be used to simulate distributed deployment.

# CPM Lab

* See also [CPM documentation](https://cpm.embedded.rwth-aachen.de/doc/)
* Make sure that the main computer in the CPM Lab is configured to use MATLAB R2023a
  * you can change version by:
  * `cd ~/.local/bin`
  * `ln -sf /usr/local/MATLAB/R2023a/bin/matlab matlab `

### No Parallel Computation / Distributed Execution

* Select main.m as script in the LCC
* The UI window opens and you can enter the vehicle ids
* Click start to run the experiment

### Distributed Deployment on the Nucs
 * Run main.m and select your config. The config file will be written to the disk and copied to the Nucs.
   * Select the amount of vehicles
   * activate Parallel Computation
   * enter all vehicle ids


 * main.m will delete all generated ros2 msgs if Lab mode + parallel execution is selected, because the generated msg must not be copied to the NUCS
 * Select MATLAB R2023a as version (See [Specifying the matlab version for HLCs on the NUCs](https://cpm.embedded.rwth-aachen.de/doc/pages/viewpage.action?pageId=25395201))


 * start the NUCS

 * make sure enough Nucs are shown as registered in LCC


 * select main_distributed.m as script in LCC and click deploy in LCC.

# Unified Testbed Interface

In order to generalize the experiments for model-scale labs, the Unified Testbed Interface (UTI) defines a standardized API on which basis High Level Controllers and scenarios can be programmed lab-independently. By selecting the UTI environment, communication is done with the currently available lab via this ROS2-based interface. In order to run an experiment, follow these steps:
1. Launch all ROS2 nodes necessary for the used lab.
2. Run matlab's `main` program and select `Unified Testbed Interface` as environment. You can now switch between the following two scenario:
    1. Lanelet2: This is a scenario working with the cloverleaf map typically used in the CPM Lab. This map is sent to the lab, has to pass the lab-conformity check, and can then be used for experiments. In this scenario, a path is generated randomly by using the lanelet2 library and mex.
    2. Lab Default: In the first step, the default lanelet2 map of the currently accessed lab is retrieved. On this retrieved map, a path is generated randomly as for the Lanelet2 scenario.


# Debug Parallel Computation

Matlab Parallel Computation Toolbox doesn't support debugging. However, you can select the config by executing main.m and activate parallel compuatation in the UI. The scenario will be written to the disk. Afterwards, you can open multiple matlab instances on your PC and execute main_distributed(1 vehicle id). You have to open as many Matlab instances as vehicles selected in the UI.

If there is an error with `ros2genmsg()` execute `ros2genmsg(commun/cust1)` manually in 1 of the Matlab instances and try again. Probably there is an issue with multiple processes trying to write to the same file.


# UI
-	 Parallel Computation / Distributed Execution is only relevant for Pb non-coop scenarios. If Distributed Execution is used, the program will create a HLC for each vehicle. Otherwise 1 HLC controls all vehicles.

- You can set the desired vehicle ids in the respective Box in the UI. The number of entered vehicle ids must equal the selected vehicle amount. If you leave the Box empty, some default vehicle ids will be used.


# Troubleshooting
- If you encounter problems like `"MATLAB has experienced a low-level graphics error, and may not have drawn correctly."`, close Matlab and restart again by using `matlab -softwareopengl`.
- If you run into problems with the standard c++ library libstdc++ try one of the ways described here: https://de.mathworks.com/matlabcentral/answers/329796-issue-with-libstdc-so-6 . "1. Renaming the libstdc++.so.6 library file so that MATLAB cannot find it and is forced to use the system's version of the library." worked for me.