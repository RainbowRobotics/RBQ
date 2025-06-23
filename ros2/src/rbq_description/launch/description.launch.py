# Copyright (c) 2024 Rainbow-Robotics. All rights reserved.

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> launch.LaunchDescription:
    pkg_share = FindPackageShare(package="rbq_description").find("rbq_description")
    default_model_path = os.path.join(pkg_share, "urdf/rbq.urdf.xacro")
    default_rviz2_path = os.path.join(pkg_share, "rviz/viz_rbq.rviz")
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui", default_value="True", description="Flag to enable joint_state_publisher_gui"
            ),
            DeclareLaunchArgument(
                name="model", default_value=default_model_path, description="Absolute path to robot urdf file"
            ),
            DeclareLaunchArgument(
                name="rvizconfig", default_value=default_rviz2_path, description="Absolute path to rviz config file"
            ),
            DeclareLaunchArgument(name="arm", default_value="False", description="Flag to enable arm"),
            DeclareLaunchArgument(
                "tf_prefix", default_value='"rbq"', description="Apply namespace prefix to robot links and joints"
            ),
            DeclareLaunchArgument("namespace", default_value="rbq", description="Namespace for robot tf topic"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                LaunchConfiguration("model"),
                                " arm:=",
                                LaunchConfiguration("arm"),
                                " tf_prefix:=",
                                LaunchConfiguration("tf_prefix"),
                            ]
                        )
                    }
                ],
                namespace=LaunchConfiguration("namespace"),
                remappings=[
                    ('/joint_states', '/robot_state/joint_state')
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
                namespace=LaunchConfiguration("namespace"),
                remappings=[
                    ('/joint_states', '/robot_state/joint_state')
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d" + default_rviz2_path],
            ),
        ]
    )
