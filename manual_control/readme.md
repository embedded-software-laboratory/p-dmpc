# Manual Control

### Requirements
* Ubuntu 18.04
* ROS2 eloquent
* Manual controlling device(s)
    * Check functionality with `jstest-gtk`
    * To enable force feedback: Logitech G29 Driving Force Racing Wheel

### Setup
1. Checkout the [installation script](install.sh). Currently not executable, but needs to be executed line by line in the terminal.
2. Raspberry Pi for camera stream: Download and install the software from: https://nerdhut.de/2018/12/17/low-latency-and-high-fps-camera-stream-with-raspberry-pi/

### Config
* Force Feedback: check whether the device name is correct. Find Logitech G29 Driving Force Racing Wheel and check if handler (e.g. event7) matches entry in [yaml file](launch_g29_force_feedback.xml).
    ```bash
    cat /proc/bus/input/devices
    ```
* Camera stream:
    1. Connect Raspberry Pi with Lab-PC
        ```bash
        ssh pi@192.168.1.158
        pi@192.168.1.158's password: camera
        ```
    2. Setup livestream
        ```bash
        cd camera-stream/
        python3 ./server.py
        ```

### Run
* Access camera stream in browser: 192.168.1.158:8080
* Control a vehicle:
    ```matlab
    startup()
    manual_control(vehicle_id, input_device_id, control_mode)
    ```