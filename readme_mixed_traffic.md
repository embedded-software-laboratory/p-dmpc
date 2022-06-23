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
* Install joy package: https://index.ros.org/p/joy/
  ```js
  sudo apt-get install ros-eloquent-joy ros-eloquent-joystick-drivers
  ```
* To enable force feedback using Logitech G29: Install https://github.com/kuriatsu/ros-g29-force-feedback
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


