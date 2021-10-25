# usma_mavros

These instructions are for operating a small, multirotor UAS running a PX4 flight control unit or similar variant. They also include setup for a ROS interface using mavlink and mavros.  The companion computer can be a number of embedded devices such as a NUC, Odroid, or RPi while using a ground station PC connected via WiFi and a telemetry radio.

## Outdoor Operations

### PX4 Instructions
1. We recommend first starting with a tethered configuration (power and communications). A Linux computer can connect to the PX4 using the USB port. Follow the [PX4 development guide](https://docs.px4.io/v1.9.0/en/config/). See your instructor with questions.
2. Configure the PX4 using [QGroundControl](http://qgroundcontrol.com/) (download latest [firmware](https://github.com/PX4/Firmware/releases/), set up flight modes, etc).
  - The `SERIAL2_BAUD` parameter is set to 921600. This paramater is the Telemetry 2 port on the PX4.
3. Once your PX4 is configured and calibrated, it can connect to a companion computer through an FTDI adapter to the Telemetry 2 port on the PX4 as described [here](http://dev.px4.io/v1.9.0/en/companion_computer/pixhawk_companion.html).
4. These instructions assume the latest stable version of the firmware ([v1.10.0 as of March 2020](https://github.com/PX4/Firmware/releases/tag/v1.10.0)).  Note: If the /tag/v*.*.* page does not have .px4 files, go to the next available stable release.  For the Pixhawk v2.1 cube, you need fmu-v3, e.g. px4_fmu-v3_default.px4

### Computer Instructions
1. Install the [mavros package](http://wiki.ros.org/mavros) and dependencies (assuming the stable version of ROS is used).
2. Clone usma_mavros
 - `cd ~/catkin_ws/src`
 - `git clone https://github.com/westpoint-robotics/usma_mavros`
3. Build
 - `cd ~/catkin_ws/`
 - `catkin_make -DMAVLINK_DIALECT=common`
4. Configure mavros on computer:
 - In `px4.launch`, 
 - The computer is connecting through `ttyUSB0:921600` instead of `ttyACM0:57600`.
 - Chage the `gcs_url` argument default to `default="udp://:14556@192.168.XX.XX:14550"` to match the IP of the ground control station (which may be determined using `ifconfig`, or `ip a`).
 - QGC can connect to the autopilot using the Default UDP link.  
5. Execute mavros
 - `roslaunch usma_mavros px4.launch`
 - Check that there is a heartbeat with the PX4. 
 - `[INFO] [1573276705.686808500]: CON: Got HEARTBEAT, connected. FCU: ArduPilot`
 
----- Need to update indoor operations and below ------

## Indoor Operations

1. We recommend first starting with a tethered configuration (power and communications). A Linux computer can connect to the PX4 using a USB extension and FTDI adapter to the Telemetry 2 port on the PX4 as described [here](http://ardupilot.org/dev/docs/odroid-via-mavlink.html).  For initial testing and configuration, it is possible to connect to the PX4's micro USB port.

2. For general manual control, configure the PX4 using [QGroundControl](qgroundcontrol.org/) (download latest firmware, set up flight modes, etc).

3. If operating in mocap, follow the [usma_optitrack](https://github.com/westpoint-robotics/usma_optitrack) instructions for set up motion capture.

   - Configure the PX4 to operate in mocap using [external position estimation](https://dev.px4.io/en/ros/external_position_estimation.html).
 
   - Instructions are included to build from source. The firmware can also be downloaded [here](http://px4-travis.s3.amazonaws.com/Firmware/stable/px4fmu-v2_lpe.px4).
   
   - Transmitter settings can be found [here](). At a minimum, the RC transmitter should have manual, position control, offboard control, and a kill switch. 
   
   - First achieve position control of the vehicle using the transmitter.
   
   - Here is a summary of firmware parameters:

      -- The `sys_companion` field is set 921600.
      -- `ATT_EXT_HDG_M` parameter is set to 2.

     
    
 








## Outdoor Operations


### PX4 Instructions
1. We recommend first starting with a tethered configuration (power and communications). A Linux computer can connect to the PX4 using a USB extension and FTDI adapter to the Telemetry 2 port on the PX4 as described [here] (http://ardupilot.org/dev/docs/odroid-via-mavlink.html).  
2. Configure PX4 using [QGroundControl](qgroundcontrol.org/) (download latest firmware, set up flight modes, etc). 
 - If operating in mocap, follow the [usma_optitrack](https://github.com/westpoint-robotics/usma_optitrack) instructions for set up motion capture.
 - Configure the PX4 to operate in mocap using [external position] (http://dev.px4.io/external-position.html).
 - The `sys_companion` field is set 921600.
 - `ATT_EXT_HDG_M` parameter is set to 1 (when using vision_pose topic).
 - `CBRK_NO_VISION` parameter set to 0.
 - Indoor navigation is only achieved by using the external headings from mocap.
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
5. Configure mavros on computer:
 - Note some variations to the mavros configuration:
 - Using QGC, the `sys_companion` field is set 921600 so that autopilot is enabled with a higher baud rate to communicate with the computer.
 - In `px4.launch`, 
 - The computer is connecting through `ttyUSB0:921600` instead of `ttyACM0:57600`.
 - Chage the `gcs_url` argument default to `default="udp://:14556@192.168.200.XX:14550"` to match the IP of the ground control station (which may be determined using `ifconfig`, or `ip a`).
 - QGC can connect to the autopilot using the Default UDP link.  
 - If using mocap, ensure that the `vision_pose_estimate` plugin is enabled and NOT blacklisted.
6. Execute mavros
 - `roslaunch usma_mavros px4.launch`
 - Check that there is a heartbeat with the PX4.
 - Check the debug messages for a valid vision estimate: `FCU: [inav] VISION estimate valid`
 - Echo the FCU local position to check for propoer coordinate frame conversions: `rostopic echo /mavros/local_position/local`

<a href="url"><img src="https://github.com/westpoint-robotics/usma_vtol/blob/master/setup.png" align="center" width="500" ></a>


### Instructions to set up a Raspberry Pi 2

1. [Install Ubuntu 14.04 LTS] (https://wiki.ubuntu.com/ARM/RaspberryPi)
 - [These instructions are helpful as well] (https://www.raspberrypi.org/documentation/installation/installing-images/linux.md)
2. Connect your RPi2 using wired Ethernet. 
3. Install linux-firmware drivers to enable wifi.
 - `sudo apt-get install linux-firmware`
 - `sudo apt-get install wicd`
4. [Setup WiFi on your device] (https://help.ubuntu.com/community/NetworkConfigurationCommandLine/Automatic)
 - [Another reference] (https://learn.adafruit.com/adafruits-raspberry-pi-lesson-3-network-setup/setting-up-wifi-with-occidentalis)
 - `$ sudo nano /etc/network/interfaces`
```
auto wlan0
allow-hotplug wlan0
iface wlan0 inet dhcp
  wpa-ssid "EECSDS3"
  wpa-psk "accessgranted"
```
5. [Install ROS] (http://wiki.ros.org/indigo/Installation/UbuntuARM)
6
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

#### Offboard Tracking  
1. set up ROS_MASTER_URI and IP on all the terminals used in tracking 
2. Run <code> roslaunch mavros px4.launch </code> on onboard computer
3. on remote host,set up ROS_MASTER_URI and IP and run <code> roslaunch mocap_optitrack mocap.launch </code>
4. on different terminal,set up ROS_MASTER_URI and IP and run  <code> roslaunch mavros_extras teleop_track.launch </code>
5. Joystick button map is listed in f710_joy.yaml( for take-off ,land, track button reference) 



# New instructions

## Quad rotor naming

All tail numbers must be unique

- Mocap rigid body naming  -  f450_<TAIL_NUM>
- Hostname  -  odroid-<TAIL_NUM>

# PX4 setup
- Flash the firmware located in the config folder
- Load the params file in the config folder
- Calibrate all sensors

# Odroid setup
During OS installation
- Hostname  -  odroid-<TAIL_NUM>

After OS install
```
echo "192.168.200.88 ros304" | sudo tee -a /etc/hosts
```

After ROS install
- clone and make mavros from source
- make sure to compile with one job otherwise it will most likely fail

```
cd ~/catkin_ws
catkin_make -j1
```

Generate and share SSH key (no password)
```
ssh-keygen
cat ~/.ssh/id_rsa.pub
```

Copy and paste this into the base station's authorized_keys file in a new line

# Base station setup
Only once
```
echo  "export ROSLAUNCH_SSH_UNKNOWN=1" >> ~/.bashrc
echo  "export ROS_MASTER_URI=http://ros304:11311" >> ~/.bashrc
```

For each Odroid
```
echo "<IP_ADDRESS> odroid-<TAIL_NUM" | sudo tee -a /etc/hosts
```

An ssh key should already be setup. Share with the odroid
```
cat ~/.ssh/id_rsa.pub
```
Copy and paste this into the Odroid's authorized_keys file in a new line


# How to run

Make sure a BATTERY is plugged in

Terminal A
```
roslaunch usma_mavros base_station.launch
```

Terminal B
```
roslaunch usma_mavros lpe_mavros_quad.launch TAIL_NUM:=131
```

Terminal C
```
rosrun usma_mavros test_flight __ns:=f450_131
```

# Mavros Setup Insutruction - Fall 2021

## Companion Computer Configuration
Configure an sd card with ubuntu 20.04. I can be downloaded [here](https://ubuntu-mate.org/download/)
