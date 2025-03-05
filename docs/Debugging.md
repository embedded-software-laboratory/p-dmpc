# Debugging

### Parallel Computation Toolbox

- The Parallel Computation Toolbox does not support Debugging. However, debug messages can be printed to the console.
- One can use the sequential PB Controller for Debugging.
- One can also use `main_distributed` to debug the parallel PB Controller. This works for the CPM Lab and also for local simulations:
  1. Start `main.m` and config your scenario. Select parallel computation. The scenario file will be written to disk
  2. By default, the software will start a parallel pool. Abort this operation.
  3. Open as many MATLAB sessions as vehicles selected in step 1)
  4. Run `main_distributed(i_vehicle)` for each vehicle in one of the MATLAB session.
  5. Select breakpoints for the vehicles as required. If you want to debug only a specific vehicle, you might want to disable the timeouts in Predictions and/or Traffic Communication classes to avoid the other vehicles running into timeouts.

### Debug in Lab

- If your are not using the NUCs, the debugging hints from above apply.
- Otherwise:

1. Setting breakpoints in Matlab on the NUCs is not possible yet.
2. The Output of the Matlab console is written to a log file. Furthermore, the `results.mat` are written to the disk. These data can be copied from the NUCs to the main computer by use of the `copy_data_from_nuc` script, which you can find on the desktop of the main computer.
3. Setting breakpoints in the C++ Code on the Nucs was not tested yet. It might be possible by using the Visual Studio Remotetools for Linux.
4. If you want to debug a specific vehicle, it can be helpful to run only this vehicle on the main computer (in order to set breakpoints) and the remaining ones on the NUCs. The LCC doesn't provide this functionality. Thus, you have to update and execute the deploy scripts of the LCC software manually.
