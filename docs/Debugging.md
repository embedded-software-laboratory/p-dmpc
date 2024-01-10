## Debugging

### Windows (Clion / Visual Studio)

#### Pre-Requisites
- Install Visual Studio (https://visualstudio.microsoft.com/de/downloads/)
- Set your compiler to the Visual Studio C++ Compiler
- Open the Project as CMake Project

#### Steps
1. Build the Project in Debug Mode (-g option). In CMake you can use
> set(CMAKE_BUILD_TYPE Debug) to build with debug symbols
2. This should create the .mexw64 file and additionally a .pdb file
3. Start Matlab (the same version of Matlab that you generated the mex file)
4. Press "Attach..." or "Attach to Process" in the IDE and select MATLAB.exe or the MATLAB mexhost process, if you use out of process execution
5. Wait until the connection is established
6. You can now set breakpoints in Clion / Visual Studio and execute the mex file. (You can ignore the warning: breakpoint not reachable)
7. When you compile again just use "clear mex" function, so that linking won't fail

### Linux (Clion)

#### Pre-Requisites
- Install g++ and gdb
- Set your compiler to g++
- Open the Project as CMake Project

#### Steps
1. Build the Project in Debug Mode (-g option). In CMake you can use
> set(CMAKE_BUILD_TYPE Debug) to build with debug symbols
2. This should create the .mexa64 file
3. Start Matlab (the same version of Matlab that you generated the mex file)
4. Press "Attach to Process" in the IDE, select MATLAB and attach with GDB
5. Wait until the connection is established
6. You can now set breakpoints in Clion and execute the mex file.
7. Most of the time, if there is a SIGSEGV you can skip over it... <3

### Linux (VS Code)

#### Pre-Requisites
- Install g++ and gdb
- Set your compiler to g++
- Open the Project as CMake Project
- Configure launch.json like for example:

- `{
    "configurations": [
    {   "name": "(gdb) Attach",
        "type": cppdbg",
        "request": "attach",
        "program": <pathToMatlab>,
        "processId": "${command:pickProcess}",
        "MIMode" "gdb",
        "setupCommands": [
            {
                "text": "handle SIGSEGV nostop"
            },
            {
                "text": "handle SIGSEGV noprint"
            }
        ]
    }
    ]
}`

#### Steps
1. Build the Project in Debug Mode (-g option). In CMake you can use
> set(CMAKE_BUILD_TYPE Debug) to build with debug symbols
2. This should create the .mexa64 file
3. Start Matlab (the same version of Matlab that you generated the mex file)
4. Press "Start Debugging" in the IDE, select MATLAB (or MATLAB mexhost) and attach with GDB
5. Wait until the connection is established
6. You can now set breakpoints in VS Code and execute the mex file.

### Parallel Computation Toolbox
 - The Parallel Computation Toolbox does not support Debugging. However, debug messages can be printed to the console.
 - One can use the sequential PB Controller for Debugging.
 - One can also use `main_distributed` to debug the parallel PB Controller. This works for the CPM Lab and also for local simulations:
    1. Start `main.m` and config your scenario. Select parallel computation. The scenario file will be written to disk
    2. By default, the software will start a parallel pool. Abort this operation.
    3. Open as many MATLAB sessions as vehicles selected in step 1)
    4. Run `main_distributed(veh_id)` for each vehilce in one of the MATLAB session.
    5. Select breakpoints for the vehicles as required. If you want to debug only a specific vehicle, you might want to disbale the timeouts in Predicitons and/or Traffic Comuniction classes to avoid the other vehicles running into timeouts.

### Debug in Lab
 - If your are not using the NUCs, the debugging hints from above apply.
 - Otherwise: 
  1. Setting breakpoints in Matlab on the NUCs is not possible yet.
  2. The Output of the Matlab console is written to a log file. Furthermore, the `results.mat` are written to the disk. These data can be copied from the NUCs to the main computer by use of the `copy_data_from_nuc` script, which you can find on the desktop of the main computer.
  3. Setting breakpoints in the C++ Code on the Nucs was not tested yet. It might be possible by using the Visual Studio Remotetools for Linux.
  4. If you want to debug a specific vehicle, it can be helpful to run only this vehicle on the main computer (in order to set breakpoints) and the remaining ones on the NUCs. The LCC doesn't provide this functionality. Thus, you have to update and execute the deploy scripts of the LCC software manually.
