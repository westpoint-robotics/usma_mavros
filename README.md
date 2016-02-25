# usma_mavros
Instructions on how to configure an embedded computer and PX4 to run mavros.

## Setup embedded computer (Raspberry Pi 2)
1. Assume Ubuntu 14.04 and ROS Indigo installed and a catkin_ws created
2. Install mavros package
 * `sudo apt-get install ros-indigo-mavros`
3. Clone usma_mavros
 * `cd ~/catkin_ws/src`
 * `git clone https://github.com/westpoint-robotics/usma_mavros`
4. Set up embedded computer to match MASTER_URI of base station
 * $ `cd ~`
 * $ `vim .bashrc` (or your preferred editor)
 * Add this export to the end of your .bashrc file that matches the IP of your master machine (i.e. the basestation):
  * export ROS_MASTER_URI=http://mavros:11311
 * Both the client and server should have this export in their .bashrc
 * Edit your /etc/hosts to include the computer name and IP address for the other computer similar to these [instructions] (http://www.faqs.org/docs/securing/chap9sec95.html) 
 * Other references:
  * http://wiki.ros.org/ROS/Tutorials/MultipleMachines
  * http://wiki.ros.org/ROS/NetworkSetup
5. Run mavros on the embedded computer:
 * `roslaunch usma_mavros px4.launch`
