# Simulation
* execute `main()` and choose "Simulation" as environment

# CPM Lab
* See also [CPM documentation](https://cpm.embedded.rwth-aachen.de/doc/)
* MATLAB
    * Build a `Config` object (e.g., `config`) with `config.environment = Environment.CpmLab`
    * Run `main(config, vehicle_ids = [<<active vehicle ids>>])`
* Lab Control Center
    * Set middleware period
    * Deploy without script name and with *deactivated* distributed deployment
    * Wait for middleware to show up (note: sometimes not all NUCs register; experiment might work anyways)
    * Start timer
    * Note: No HLC will show up as online, since we kill the process responsible for the heartbeat on the NUCs

# Debug Thread Parallel Computation

Matlab Parallel Computation Toolbox doesn't support debugging. However, you can select the config by executing main.m and activate parallel computation in the UI. The scenario will be written to the disk. Afterwards, you can open a Matlab for every vehicle and execute `main_distributed(i_vehicle)` within each.

# Troubleshooting
- If you encounter problems like `"MATLAB has experienced a low-level graphics error, and may not have drawn correctly."`, close Matlab and restart again by using `matlab -softwareopengl`.
- If you run into problems with the standard c++ library libstdc++ try one of the ways described here: https://de.mathworks.com/matlabcentral/answers/329796-issue-with-libstdc-so-6 . "1. Renaming the libstdc++.so.6 library file so that MATLAB cannot find it and is forced to use the system's version of the library." worked for me.