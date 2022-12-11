# MavROS Docker Image

The folder contains code to build a px4-ready mavros docker image.

It is designed to work with virtualized px4 images, such as [px4-gazebo-headless](https://github.com/JonasVautherin/px4-gazebo-headless)

Once running, it will provide a ROS node that can interface with PX4 via UDP

## Build the PX4-MavRos container

```
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
docker run -e ROS_MASTER_URI='http://localhost:11311' -e FCUURL='udp://:14540@127.0.0.1:14557' -e STARTUP_DELAT=10 -it usma-robotics/mavros 
```

## Simulation examples



