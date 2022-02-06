# ROS 2 & PX4 instalation guide

# PX4 ROS 2 bridge

User guide for [PX4 ROS2 bridge](https://docs.px4.io/master/en/ros/ros2_comm.html)

# ROS 2 & PX4 instalation guide

  * [Install Fast DDS](#install-fast-dds)
  * [Install ROS2](#install-ros-2-foxy)
  * [Create & build ROS 2 workspace](#create-&-build-ros-2-workspace)

## Install Fast DDS

[Fast DDS Installation Guide](https://docs.px4.io/master/en/dev_setup/fast-dds-installation.html)

> **NOTE:**
> 
> You do not have to install Fast DDS if you have ROS 2 Foxy (Ubuntu 20.04) installed.
> This means you just need to install Fast-RTPS-Gen and have your ROS 2 environment sourced `(source /opt/ros/<distro>/setup.bash)` in order to be able to compile the rtps targets in the PX4-Autopilot repo.

### Java JDK

To install open source java JDK

```bash
sudo apt-get install openjdk-11-jdk
```

### Gradle

Install gradle according to [link](https://linuxize.com/post/how-to-install-gradle-on-ubuntu-20-04/)

> **WARNING:**
>
> Do not install Gradle version 7 or higher. The recommended version is 6.3.

Download the gradle

```bash
wget https://services.gradle.org/distributions/gradle-6.3-bin.zip -P /tmp
```

Once the download is completed, unzip the file in the ``/opt/gradle`` directory:

```bash
sudo unzip -d /opt/gradle /tmp/gradle-6.3-bin.zip
```

Gradle is regularly updated with security patches and new features. To have more control over versions and updates, we’ll create a symbolic link named latest, which points to the Gradle installation directory:

```bash
sudo ln -s /opt/gradle/gradle-6.3 /opt/gradle/latest
```

Later, when upgrading Gradle, unpack the newer version and change the symlink to point to it.

#### Setting up the Environment Variables 

We need to add the Gradle bin directory to the system PATH environment variable. To do so, open your text editor and create a new file named gradle.sh inside of the ``/etc/profile.d/`` directory.

```bash
sudo nano /etc/profile.d/gradle.sh
```

Paste following configuration

```bash
export GRADLE_HOME=/opt/gradle/latest
export PATH=${GRADLE_HOME}/bin:${PATH}
```

Save and close the file. This script will be sourced at shell startup.

Make the script executable

```bash
sudo chmod +x /etc/profile.d/gradle.sh
```

Load the environment variables in the current shell session using the source command:

```bash
source /etc/profile.d/gradle.sh
```

If you don’t want to have to source the setup file every time you open a new shell, then you can add the command to your shell startup script:

```bash
echo "source /etc/profile.d/gradle.sh" >> ~/.bashrc
```

To validate that Gradle is installed properly use command:

```bash
gradle -v
```

### Fast-RTPS-Gen


Fast-RTPS-Gen is the Fast RTPS (DDS) IDL code generator tool. It should be installed after Fast RTPS (DDS) and made sure the fastrtpsgen application is in your PATH. You can check with which fastrtpsgen.

Then install Fast-RTPS-Gen 1.0.4 (Gradle is required for this):

```bash
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
    && cd ~/Fast-RTPS-Gen \
    && gradle assemble \
    && sudo env "PATH=$PATH" gradle install
```

## Install ROS 2 Foxy

1. [Install ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

2. The install process should also install the colcon build tools, but in case that doesn't happen, you can install the tools manually:

```bash
sudo apt install python3-colcon-common-extensions
```

3. ``eigen3_cmake_module`` is also required, since Eigen3 is used on the transforms library:

```bash
sudo apt install ros-foxy-eigen3-cmake-module
```

1. Some Python dependencies must also be installed (using pip or apt):

```bash
sudo pip3 install -U empy pyros-genmsg setuptools
```

## Create & build ROS 2 workspace

1. Clone git repository
```bash
git clone https://github.com/DavidLindtner/px4_ros2_sim.git
cd px4_ros2_sim
git submodule init
git submodule update
```
2. Use the `build_ros2_workspace.bash` script to build the ROS 2 workspace (including `px4_ros_com` and `px4_msgs`).
```bash
cd px4_ros_com_ros2/src/px4_ros_com/scripts
source build_ros2_workspace.bash
```
3. You will need to run this command on every new shell you open to have access to the ROS 2 workspace
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
```
Or put it into .bashrc
```bash
echo "source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash" >> ~/.bashrc
```

```bash
colcon build --allow-overriding px4_missions px4_msgs px4_ros_com
```
