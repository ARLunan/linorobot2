# Linorobot2gz
This **README-gz-sim-ros-gz_INSTALLATION.md** in this repository specfically describes using gz\_sim Harmonic and compatible ros\_gz. This linorobot2gz respository is based on a fork of the Linorobot2 repository [[https://github.com/chazyman/linorobot2] (Version for Ubuntu 22.04/ROS 2 Humble, that was migrated from Classic gazebo to support Gazebo Ignition Fortress. This Linorobot2gz repository code revisions support ROS2 Humble gz-sim Harmonic/ros-gz.  In the file linorobot2gz/gazebo.launch.py, several instances of ignition are changed to gz. 

**Installation**: This Linorobot2gz repository requires the installation of the Harmonic version of gz-sim (from Binary on Intel x86 and arm64 machines) and ros-gz (Binary on Intel x86 and from Source on arm64 machines such as MAC M1 Virtual).  

**1) gz-sim**: [https://gazebosim.org/docs/harmonic/install_ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu). 
	
After installing necessary tools and osrfoundation gpg and keys, run 
	**$ sudo apt install gz-harmonic** and	
	**$ export GZ_VERSION=harmonic** 
	for the current Terminal instance or put this statement in the .bashrc file. 
	
To check for a valid installation run  
**$ gz sim --version
Gazebo Sim, version 8.0.0 should be displayed**

and run  
**$ gz sim --shapes.sdf**  
should display a Gazebo Gui with colored shapes.


**2) ros-gz**: 
[https://gazebosim.org/docs/harmonic/ros_installation](gazebosim.org/docs/harmonic/ros_installation). 
	
**Binary**: $ sudo apt install ros-humble-gz-harmonic 
	
**Source**: Refer to [https://github.com/gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)

If not already done so, add [https://packages.ros.org/](https://packages.ros.org/)

**Configure ROS2 colcon workspace: e.g.**

rosgz\_ws/src 

git clone [https:github.com/gazebosim/ros_gz/tree/humble](https://github.com/gazebosim/ros_gz/tree/humble) **Note the humble Branch - NOT the default ROS 2**

**Revise ros\_gz\_sim\_demos**, ros\_gz\_sim\_demos.dsv.in 
contains IGN\_GAZEBO\_RESOURCE\_PATH. It might need to be updated to GZ\_SIM\_RESOURCE\_PATH. 

Before compiling: $ export MAKEFLAGS="-j 1" 

Build (using the optional following options depending on your machine Memory and CPU Cores specifications):

Enter workspace root and build:

cd ~/rosgz\_ws

colcon build —executor sequential —parallel-workers=1

When complete, source workspace root

. install/setup.bash

Running $ ros2 pkg list|grep gz should display:

ros\_gz  
ros\_gz_bridge  
ros\_gz\_example\_application  
ros\_gz\_example\_bringup  
ros\_gz_example_description  
ros\_gz_example_gazebo  ros\_gz\_image  
ros\_gz\_interfaces  
ros\_gz\_sim  
ros\_gz\_sim_demos

**Now following Linorobot2gz repository README.md**: Bulld and Install the Linorobot2gz packages in a Linorobot2\_ws workspace on the Development Desktop Computer. Don't forget to source the workspace root with . install/setup.bash or put the "source ~/linorobot2gz_ws/install/setup.bash in the .bashrc and source . .bashrc to create an overlay.

**2.1 Install linorobot2 Package**

Install linorobot2 package on the host machine:

cd <host_machine_ws>
git clone [https://github.com/ARLunan/linorobot2gz](https://github.com/ARLunan/linorobot2gz) 
    
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent

colcon build  
source install/setup.bash

**1.1b Using Gazebo:**

**ros2 launch linorobot2_gazebo gazebo.launch.py

The **Gazebo Gui* will open displaying the Linorobot2 Robot model.

**Postscript**:
For an additional educational experience, explore this repository: [https://github.com/gazebosim/ros\_gz\_project\_template](https://github.com/gazebosim/ros_gz_project_template)

After building the packages following the colcon options described previously, and sourcing the workspace root, run

**ros2 launch ros\_gz\_example\_bringup diff\_drive.launch.py**

A RViz and Gazebo Gui should open displaying hte diff\_driver robot model.