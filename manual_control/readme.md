# Mixed Traffic Scenario
Assistance to run mixed traffic scenarios embedded in the Receding Horizon Graph Search (RHGS) repository, a MATLAB implementation of a graph-based receding horizon trajectory planner.

## Requirements
* The code is developed with MATLAB R2020a.
* Ubuntu 18.04
* ROS2 eloquent
* Manual controlling device(s)
  * There are up to two controlling devices supported
  * To enable force feedback: Logitech G29 Driving Force Racing Wheel required


## Setup
### Joy package
* Install joy package: https://index.ros.org/p/joy/
  ```js
  sudo apt-get install ros-eloquent-joy ros-eloquent-joystick-drivers
  ```

### CPM Lab
1. check whether joystick settings are correct:
    ```js
    jstest-gtk
    ```
    Steering wheel has to be device: /dev/input/js0, gamepad has to be device: /dev/input/js1 \
    If incorrect, then restart computer and connect steering wheel before gamepad \
    (For default event handler, connect steering wheel to the right USB-port in the second row)
2. Checkout correct CPM Lab branch
    ```js
    cd dev/software
    git checkout integrate_steering_wheel_david
    ./build_all.bash
    ```
3. Open lab_control_center
  * delete script path
  * change middleware period to 200ms (Parameters -> middleware_period)
### Force Feedback
* To enable force feedback using Logitech G29: Install https://github.com/kuriatsu/ros-g29-force-feedback \
  Yaml Parameters (install/ros_g29_force_feedback/share/ros_g29_force_feedback/config/g29.yaml):
  ```
  device_name: "/dev/input/event7"
  loop_rate: 0.01
  max_torque: 1.0
  min_torque: 0.2
  brake_torque: 0.2
  brake_position: 0.1
  auto_centering_max_torque: 0.3
  auto_centering_max_position: 0.1
  eps: 0.01
  auto_centering: false
  ```
  * Troubleshooting: 
    * check whether the device name is correct: 
      ```js
      cat /proc/bus/input/devices
      ```
      Find Logitech G29 Driving Force Racing Wheel and check if handler (e.g. event7) matches entry in yaml file
### Camera livestream
* e.g. for Raspberry Pi V2 camera and Raspberry Pi 4
1. Download and install the software from: https://nerdhut.de/2018/12/17/low-latency-and-high-fps-camera-stream-with-raspberry-pi/

2. Connect Raspberry Pi with Lab-PC
  ```js
  ssh pi@192.168.1.158
  pi@192.168.1.158's password: camera
  ```

3. Setup livestream
  ```js
  cd camera-stream/
  python3 ./server.py
  ```

4. Access livestream in browser: 192.168.1.158:8080

## Run
* To run a scenario:
  ```matlab
  startup()
  startup_mixed_traffic(number_of_controllers, force_feedback_enabled)
  main(Vehicle_IDs)
  ```
  Input Parameters:
   * number_of_controllers: set 1 for a single manual controlling device, 2 for two manual controlling devices
   * force_feedback_enabled: set true to enable force feedback
   * Vehicle_IDs: specify the IDs of the simulated vehicles
* Troubleshooting: 
    * check whether the vehicle IDs selected in the UI match the input vehicle IDs
    * check whether Commonroad is selected as scenario


## Evaluation
1. Go to the file ```main.m``` and comment out the line ```options = startOptions();``` to disable the user interface
2. * To evaluate the Guided Mode: uncomment the line ```%[options, vehicle_ids] = eval_guided_mode(1);```

      Input Parameters:
      * 1 = Collision Avoidance based on planned trajectories
      * 2 = Collision Avoidance based on reachability analsysis
   * To evaluate the Expert Mode: uncomment the line ```%[options, vehicle_ids] = eval_expert_mode(1);```

      Input Parameters:
      * 0 = No consideration of RSS
      * 1 = Consideration of RSS
3. Change ```options.is_eval = false;``` to ```options.is_eval = true;```


## Additional information
* Disable force feedback
  1. Enter ```Ctrl+C``` in the command window that opened before on ```startup_mixed_traffic(number_of_controllers, 1)```
  2. Run in matlab ```startup_mixed_traffic(number_of_controllers, 0)```
* Run Expert Mode without calling it from main function
  1. Go to the file ```CPMLab.m``` to the function ```setup(obj)``` and comment out the two marked lines to disable automatic execution of Expert Mode
  2. Open a second matlab instance
  ```matlab
  expert_mode(Vehicle_ID, force_feedback_enabled)
  ```
    * Vehicle_ID: specify the ID of the manual vehicle in Expert Mode
    * force_feedback_enabled: set true to enable force feedback
  3. In the first matlab instance run
  ```matlab
  main(Vehicle_IDs)
  ```
  (include the vehicle controlled in Expert Mode)

