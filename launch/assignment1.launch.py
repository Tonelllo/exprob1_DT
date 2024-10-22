from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    assignment_path = FindPackageShare(package="exprob_dt").find("exprob_dt")
    urdf_path = os.path.join(assignment_path, "urdf")
    worlds_path = os.path.join(assignment_path, "worlds")
    rviz_config_path = os.path.join(assignment_path, "config")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'aruco_bot',
                                   '-topic', '/robot_description'],
                        output='screen')
    return LaunchDescription([
        # SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=models_path),
        DeclareLaunchArgument(name='model', default_value=os.path.join(urdf_path, "robot.xacro"),
                                    description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,

        ExecuteProcess(
            cmd=['gazebo', '--verbose', worlds_path+'/aruco_world.world', '-s', "libgazebo_ros_factory.so"], output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path+'/rviz.rviz'],
            output='screen'),
    ])
