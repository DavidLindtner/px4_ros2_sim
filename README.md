# Simulation of Unmanned Aircrafts in a Virtual Environment

To run PX4 ROS2 drone mission simulation you need to install

  * PX4 simulation firmware with Gazebo simulator
  * ROS2 Foxy
  * PX4 - ROS 2 bridge (microRTPS agent)

To run microRTPS agent
```bash
micrortps_agent -t UDP
```

To run Gazebo simulator with PX4 SITL firmware
```bash
make px4_sitl_rtps gazebo
```

To build ROS2 packages run this command in ROS2 directory
```bash
colcon build
```
or
```bash
colcon build --packages-select px4_ros_com
```

To run ROS2 mission
```bash
ros2 run px4_ros_com droneMission
```