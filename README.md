# Simulation of Unmanned Aircrafts in a Virtual Environment

To run PX4 ROS2 drone mission simulation you need to install

  * ROS2 Foxy
  * PX4 simulation firmware with Gazebo simulator
  * PX4 - ROS 2 bridge (microRTPS agent)

To run microRTPS agent
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
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
colcon build --packages-select px4_missions
```

To run ROS2 mission
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
ros2 run px4_missions simpleMission
```
