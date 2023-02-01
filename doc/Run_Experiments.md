# Sim Lab

- execute main() and select Config in [UI](#UI) or execute main(config).
- If you are using Distributed Execution with Sim Lab options, the Matlab Parallel Computing Toolbox will be used to simulate distributed deployment.

# CPM Lab

* See also [CPM documentation](https://cpm.embedded.rwth-aachen.de/doc/)
* Make sure that the main computer in the CPM Lab is configured to use MATLAB R2022a
  * you can change version by:
  * `cd ~/.local/bin`
  * `ls -s /usr/local/MATLAB/R2022a/bin/matlab matlab `

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
 * Select MATLAB R2022a as version (See [Specifying the matlab version for HLCs on the NUCs](https://cpm.embedded.rwth-aachen.de/doc/pages/viewpage.action?pageId=25395201))


 * start the NUCS

 * make sure enough Nucs are shown as registered in LCC


 * select main_distributed.m as script in LCC and click deploy in LCC.

# Debug Parallel Computation

Matlab Parallel Computation Toolbox doesn't support debugging. However, you can select the config by executing main.m and activate parallel compuatation in the UI. The scenario will be written to the disk. Afterwards, you can open multiple matlab instances on your PC and execute main_distributed(1 vehicle id). You have to open as many Matlab instances as vehicles selected in the UI.

If there is an error with `ros2genmsg()` execute `ros2genmsg(commun/cust1)` manually in 1 of the Matlab instances and try again. Probably there is an issue with multiple processes trying to write to the same file.


# UI
-	 Parallel Computation / Distributed Execution is only relevant for Pb non-coop scenarios. If Distributed Execution is used, the program will create a HLC for each vehicle. Otherwise 1 HLC controls all vehicles.

- You can set the desired vehicle ids in the respective Box in the UI. The number of entered vehicle ids must equal the selected vehicle amount. If you leave the Box empty, some default vehicle ids will be used.

## Create Matlab Docker image
### Manually create image
Create individual Matlab Docker image (with Python, gcc, ROS):
-	Create container based on mathworks/matlab docker image. Use "docker run -it  --name MyMatlab -p 8888:8888 -p 6080:6080 -p 5901:5901 --shm-size=512M mathworks/matlab:r2022a -vnc" as a base container
-	Access the container via your browser http://hostname:6080
-	Execute matlab as root
-	Activate matlab using your license or a license server( Set env variable MLM_LICENSE_FILE to use a license server)
-	Install ros toolbox, parallel computing toolbox, Statistics and Machine Learning Toolbox
-	Install python 3.9,  cmake, gcc
-	Remove your matlab license (Navigate to matlab folder and delete license files)
-	docker commit containerID  yourNewImage to build you own Matlab Image based on the modified container
-	Tag image like your gitlab registry (docker tag SOURCE_IMAGE[:TAG] TARGET_IMAGE[:TAG])
-	Push image to gitlab container registry

### Automatically create image
A Dockerfile exists to install Matlab in a clean Ubuntu 18.04. Docker image. It was tested for Ubuntu 18.04, Matlab R2022a and the corresponding ROS Toolbox dependencies. _For other versions small changes might be necessary_.
- Set variables in ```SETTINGS``` section of the install script ```docker/install.sh```.
- Execute script ```./docker/install.sh```.
A more detailed documentation can be found in ```docker/readme.md```.
