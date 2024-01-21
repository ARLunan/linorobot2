# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        #[FindPackageShare("linorobot2_gazebo"), "worlds", "obstacles.sdf"]
        [FindPackageShare("linorobot2_gazebo"), "worlds", "empty.sdf"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
            }.items()
        ),

        # Gazebo Sim
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),        
       launch_arguments={
           #'gz_args': '-v 4 -r ' +  os.path.join(get_package_share_directory('linorobot2_gazebo'), 'worlds', 'obstacles.sdf')
           'gz_args': '-v 4 -r ' +  os.path.join(get_package_share_directory('linorobot2_gazebo'), 'worlds', 'empty.sdf')
           #'gz_args': '-r ' +  os.path.join(get_package_share_directory('linorobot2_gazebo'), 'worlds', 'playground.world')
           #'gz_args': '-v 4 -r ' +  os.path.join(get_package_share_directory('linorobot2_gazebo'), 'worlds', 'playground2.sdf')
           }.items(),
        ),

        # Spawn
        Node(
            package='ros_gz_sim',
            executable='create',
                arguments=[
                #'-world', 'playground',
                #'-world', 'playground2',            
                '-world', 'empty',
                #'-world', 'obstacles',
                '-name', 'linorobot2_2wd',
                '-topic', 'robot_description',
        ],
        output='screen',
    
        ),

        # Gz - ROS Bridge - Harmonic - Ver 8.0.0.
        Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (IGN -> ROS2)
            '/world/playground2/model/linorobot2_2wd/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/linorobot2_2wd/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Bridge Command Velocity & Odom Msg from ROS2 -> IGN 
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',      
        ],
        remappings=[
            ('/world/playground2/model/linorobot2_2wd/joint_state', '/joint_states'),
            ('/model/linorobot2_2wd/tf','/tf')                         
            #('/model/linorobot2_2wd/odometry', 'odom'),         
        ],
        output='screen'
    )

        #Node(
        #    package='linorobot2_gazebo',
        #    executable='command_timeout.py',
        #    name='command_timeout'
        #),

        #Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[
        #        {'use_sim_time': use_sim_time}, 
        #        ekf_config_path
        #    ],
        #    remappings=[("odometry/filtered", "odom")]
        #),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(joy_launch_path),
        #)
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940