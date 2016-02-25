# usma_mavros
Instructions on how to configure an embedded computer and PX4 to run mavros.

#### Setup embedded computer (Raspberry Pi 2)
1. Make these [hardware connections] (http://dev.ardupilot.com/wiki/raspberry-pi-via-mavlink/) between the RPi2 and PX4
 * The software instructions in the above link are not followed
3. Assume Ubuntu 14.04 and ROS Jade installed and a catkin_ws created
4. Setup serial port and baud rate on RPi2
 * Create a temp folder and clone this [package] (https://github.com/vooon/rarog.git)
 * Follow the installation [instructions] (https://github.com/vooon/rarog/tree/master/rarog_configs/rpi2)
   * Ignore #4 and #5  
5. Install mavros package and dependencies
 * `sudo apt-get install ros-jade-mavros ros-jade-mavlink ros-jade-diagnostic-updater ros-jade-tf2-ros ros-jade-angles ros-jade-eigen-conversions ros-jade-image-transport ros-jade-cv-bridge ros-jade-urdf ros-jade-tf`
6. Clone usma_mavros
 * `cd ~/catkin_ws/src`
 * `git clone https://github.com/westpoint-robotics/usma_mavros`
7. Build
 * `cd ~/catkin_ws/`
 * `catkin_make -DMAVLINK_DIALECT=common`
8. Set up embedded computer to match MASTER_URI of base station
 * `cd ~`
 * `vim .bashrc` (or your preferred editor)
 * Add this export to the end of your .bashrc file that matches the IP of your master machine (i.e. the basestation):
   * export ROS_MASTER_URI=http://mavros:11311
 * Both the client and server should have this export in their .bashrc
 * Edit your /etc/hosts to include the computer name and IP address for the other computer similar to these [instructions] (http://www.faqs.org/docs/securing/chap9sec95.html) 
 * Other references:
   * http://wiki.ros.org/ROS/Tutorials/MultipleMachines
    * http://wiki.ros.org/ROS/NetworkSetup
9. Run mavros on the embedded computer:
 * `roslaunch usma_mavros px4.launch`
 * Note some variations to the mavros configuration: 
   * Using QGC, the `sys_companion` field is set 921600 so that autopilot is enabled with higher baud rate to communicate with onboard computer
    * In `px4.launch`, the RPi2 is connecting through ttyAMA0:921600 instead of ttyACM0:57600
