# MavROS Docker Image

The folder contains code to build a px4-ready mavros docker image.

It is designed to work with virtualized px4 images, such as [px4-gazebo-headless](https://github.com/JonasVautherin/px4-gazebo-headless)

Once running, it will provide a ROS node that can interface with PX4 via UDP.

This is based on the [px4-dev-ros-melodic toolchain provided by px4](https://hub.docker.com/r/px4io/px4-dev-ros-melodic), and adds arguments for easy FCU specification and delayed start.
 
## Build the PX4-MavRos container

```
cd docker/
docker build -t usma-robotics/mavros .
```

## Usage

Start the container using the following:

```
docker run -it usma-robotics/mavros 
```

There are a number of optional ENV variables available:

   * ROS_MASTER_URI :  The uri of the ROS master, defaults to `http://localhost:11311`
   * FCUURL : The fcu url, defaults to `udp://:14540@127.0.0.1:14557`
   * STARTUP_DELAY: The number of seconds to delay startup, defaults to `5`

Example:

```
docker run -e ROS_MASTER_URI='http://192.168.1.10:11311' -e FCUURL='udp://:14540@192.168.1.54:14557' -e STARTUP_DELAY=10 -it usma-robotics/mavros 
```


