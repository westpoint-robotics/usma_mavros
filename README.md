# usma_mavros

### PX4 Instructions
1. We recommend first starting with a tethered configuration (power and communications). A Linux computer can connect to the PX4 using a USB extension and FTDI adapter to the Telemetry 2 port on the PX4 as described [here] (http://ardupilot.org/dev/docs/odroid-via-mavlink.html).  
2. Configure PX4 using [QGroundControl](qgroundcontrol.org/) (download latest firmware, set up flight modes, etc). 
 - If operating in mocap, follow the [usma_optitrack](https://github.com/westpoint-robotics/usma_optitrack) instructions for set up motion capture.
 - Configure the PX4 to operate in mocap using [external position] (http://dev.px4.io/external-position.html).
 - The [offboard control](http://dev.px4.io/offboard-control.html) documentation provides a good overview.

### Computer Instructions
1. Install the mavros package and dependencies (assuming ROS Indigo used).
 - `sudo apt-get install ros-indigo-mavros ros-indigo-mavlink ros-indigo-diagnostic-updater ros-indigo-tf2-ros ros-indigo-angles ros-indigo-eigen-conversions ros-indigo-image-transport ros-indigo-cv-bridge ros-indigo-urdf ros-indigo-tf ros-indigo-control-toolbox`
3. Clone usma_mavros
 - `cd ~/catkin_ws/src`
 - `git clone https://github.com/westpoint-robotics/usma_mavros`
4. Build
 - `cd ~/catkin_ws/`
 - `catkin_make -DMAVLINK_DIALECT=common`
5. Run mavros on computer:
 * `roslaunch usma_mavros px4.launch`
 * Note some variations to the mavros configuration:
   * Using QGC, the `sys_companion` field is set 921600 so that autopilot is enabled with higher baud rate to communicate with the computer.
    * In `px4.launch`, the computer is connecting through ttyUSB0:921600 instead of ttyACM0:57600

### Instructions to set up a Raspberry Pi 2
1. Make these [hardware connections] (http://dev.ardupilot.com/wiki/raspberry-pi-via-mavlink/) between the RPi2 and PX4
 - The software instructions in the above link are not followed.
2. Assume Ubuntu 14.04 and ROS Indigo installed and a catkin_ws created.
3. Setup serial port and baud rate on RPi2:
 * Create a temp folder and clone this [package] (https://github.com/vooon/rarog.git)
 * Follow the installation [instructions] (https://github.com/vooon/rarog/tree/master/rarog_configs/rpi2)
   * Ignore #4 and #5  
4. Set up embedded computer to match MASTER_URI of base station
 * `cd ~`
 * `vim .bashrc` (or your preferred editor)
 * Add this export to the end of your .bashrc file that matches the IP of your master machine (i.e. the basestation):
   * export ROS_MASTER_URI=http://192.169.200.50:11311
 * Both the client and server should have this export in their .bashrc
 * Edit your /etc/hosts to include the computer name and IP address for the other computer similar to these [instructions] (http://www.faqs.org/docs/securing/chap9sec95.html) 
 * Other references:
   * http://wiki.ros.org/ROS/Tutorials/MultipleMachines
    * http://wiki.ros.org/ROS/NetworkSetup
5. Run mavros on the embedded computer:
 * `roslaunch usma_mavros px4.launch`
 * Note some variations to the mavros configuration: 
   * Using QGC, ensure the `sys_companion` field is set to 921600 so that autopilot is enabled with higher baud rate to communicate with onboard computer
    * In `px4.launch`, the RPi2 is connecting through ttyAMA0:921600 instead of ttyACM0:57600
