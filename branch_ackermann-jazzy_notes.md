Notes that describe revisions by ARLunan to define an akermann drive/pico2/LD19/IMU MPU6050/Wheel Encoders on a Raspberry Pi4 posted on ARLUnan RossBots Github **RacerBot**. This is a varient installed 
1) on the Raspberry Pi Ubuntu 24/ROS 2 Jazzy Robot install a fork of jimdinunzio **linorobot2** repository rolling branch that uses a new branch ackermann-jazzy , and 
2) On the Raspberry Pi Controller install a fork of hippo5329 branch master **linorobot2_hardware** new branch ackermann-jazzy. Make revsions to wheel drive functions in the platformio.ini and other relevent scripts based on Ackermann Driver text in jimdinunzio repositor. 

Revisions
1. On Raspberry Pi Robot add new robot_type Ackermann Drive. Change envirionment variable in the .bashrc file to export LINOROBOT2_BASE=ackermann. 
2. Install Sensor "LD19" package on Raspberry Pi Robot. Add environment variable  LINOROBOT2_LASER_SENSOR=ld19.
3. In linorobot2 new branch ackermann-jazzy, in Package linorobot2_description rename truckasaurus_propoerties.urdf.xacro to ackermann_properties.urdf.xacro. In file ackermann_properties.urdf.xacro, revise the physical dimentions to define the RacerBot. 
4. On Linux Desktop install linorobot2_viz Package on Linux Desktop in order to visualize  
    
