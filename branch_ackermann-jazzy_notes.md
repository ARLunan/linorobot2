Notes that describe revisions by ARLunan to define an Akermann Drive/Pico/Pico2/LD19 Lidar /IMU MPU6050/Raspberry Pi Camera/Wheel Encoders on a Raspberry Pi4 posted on ARLUnan RossBots Github **RacerBot**. This is a varient installed 
1) on the Raspberry Pi 4 Ubuntu 24/ROS 2 Jazzy Robot install a fork of jimdinunzio **linorobot2** repository rolling branch that uses a new branch ackermann-jazzy , and 
2) On the Raspberry Pi $ Controller install a fork of hippo5329 branch master **linorobot2_hardware** new branch ackermann-jazzy. Make revsions to wheel drive functions in the platformio.ini and other relevent scripts based on Ackermann Driver text in jimdinunzio repositor. 

Revisions
1. On Raspberry Pi Robot add new robot_type Ackermann Drive. Change envirionment variable in the .bashrc file to set LINOROBOT2_BASE=ackermann. 
2. Install Sensor "LD19" package on Raspberry Pi Robot. Add environment variable  LINOROBOT2_LASER_SENSOR=ld19.
3. In linorobot2 new branch **ackermann-jazzy**, in Package linorobot2_description rename truckasaurus_properties.urdf.xacro to ackermann_properties.urdf.xacro. In file ackermann_properties.urdf.xacro, revise the physical dimentions to define the RacerBot. 
4. On Raspberry Pi RacerBot, to publish the URDF:
   $ ros2 launch linorobot2_description description.launch.py
5. 6. On Linux Desktop install linorobot2_viz Package on Linux Desktop in order to visualize the Robot Model. Launch: 
   - $ ros2 launch linorobot_viz robot_model.launch.py 
   - To visualize SLAM: $ ros2 launch linorobot_viz slam.launch.py, ro
   - To visualize navigation $ ros2 linorobot2_viz navigation.launch.py 
6. On Raspberry Pi 4 Install **camera_ros** package, a ROS 2 Node for libcamera supported cameras. Following installation Raspberry Pi Camera ROS .        
