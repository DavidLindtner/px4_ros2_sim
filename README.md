# Simulation of Unmanned Aircrafts in a Virtual Environment

In order to simulate drone missions with PX4 & ROS2 you need to follow steps in `install.md`

To run PX4 ROS2 drone mission simulation you need to install

  * ROS2 Foxy
  * PX4 simulation firmware with Gazebo simulator
  * PX4 - ROS 2 bridge (microRTPS agent)

## Run simulation

In order to simulate ROS 2 mission with PX4 firmware you need to run

  * PX4 simulation firmware with Gazebo simulator
  * MicroRTPS agent 
  * ROS 2 node

To run Gazebo simulator with PX4 SITL firmware you need to run this command in PX4 firmware directory.
```bash
make px4_sitl_rtps gazebo
```

To start communication bridge between ROS 2 (DDS) and PX4 (uORB) you need to source ROS 2 workspace (installed according to `install.md`) and then run microRTPS agent.
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
micrortps_agent -t UDP
```

To run ROS2 mission you need to source ROS 2 workspace and than run ROS 2 node.
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
ros2 run px4_missions simpleMission
```
