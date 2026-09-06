from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("sensor_bridge")
    default_cfg = PathJoinSubstitution([pkg, "config", "bridge.yaml"])
    cyclonedds_uri = PathJoinSubstitution([pkg, "config", "cyclonedds_gazebo.xml"])
    rviz_cfg = PathJoinSubstitution([pkg, "rviz", "rbq_sensors.rviz"])

    config_file = LaunchConfiguration("config_file")
    partition = LaunchConfiguration("partition")
    ros_domain = LaunchConfiguration("ros_domain")
    variant = LaunchConfiguration("variant")
    lidar = LaunchConfiguration("lidar")
    rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")

    # URDF (rbq10 | rbq10_wheel), one per LiDAR build so RViz draws the same housings Gazebo does.
    # CMake generates <variant>.urdf / <variant>_livox.urdf / <variant>_ouster.urdf.
    urdf_stem = PythonExpression(
        ["'", variant, "' if '", lidar, "' == 'none' else '", variant, "' + '_' + '", lidar, "'"])
    urdf = PathJoinSubstitution([pkg, "urdf", [urdf_stem, ".urdf"]])
    robot_description = ParameterValue(Command(["cat ", urdf]), value_type=str)

    # base_link -> child frame TF. Args: x y z yaw pitch roll (rad)
    def static_tf(name, x, y, z, yaw, pitch, roll, child, condition=None):
        return Node(
            package="tf2_ros", executable="static_transform_publisher", name=f"tf_{name}",
            arguments=["--x", x, "--y", y, "--z", z, "--yaw", yaw, "--pitch", pitch, "--roll", roll,
                       "--frame-id", "base_link", "--child-frame-id", child],
            parameters=[{"use_sim_time": True}],
            condition=condition,
        )

    # RViz opens only when explicitly requested and not running headless.
    rviz_on = IfCondition(PythonExpression(["'", rviz, "' == 'true' and '", headless, "' == 'false'"]))

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_cfg,
                              description="ros_gz_bridge YAML config (gz<->ROS topic map)."),
        DeclareLaunchArgument("partition", default_value="rbq",
                              description="gz-transport IGN_PARTITION; must match the running Gazebo (sim default 'rbq')."),
        DeclareLaunchArgument("ros_domain", default_value="0",
                              description="ROS_DOMAIN_ID for the bridged topics; matches the rbq_sdk sim DDS domain 0 so RViz sees both the bridged gz sensors and the VisionPublisher camera topics."),
        DeclareLaunchArgument("variant", default_value="rbq10",
                              description="Robot URDF to load: rbq10 | rbq10_wheel (matches the running Gazebo)."),
        DeclareLaunchArgument("lidar", default_value="none",
                              description="LiDAR variant for the sensor TFs: none | livox | ouster (matches the running Gazebo)."),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Also open RViz with the bundled sensor view."),
        DeclareLaunchArgument("headless", default_value="false",
                              description="When true, do not open RViz (overrides rviz; the Gazebo GUI is unaffected)."),

        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("CYCLONEDDS_URI", ["file://", cyclonedds_uri]),
        SetEnvironmentVariable("IGN_PARTITION", partition),
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain),

        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="sensor_bridge",
            output="screen",
            parameters=[{"config_file": config_file}, {"use_sim_time": True}],
        ),

        # robot_description + animated TF tree.
        Node(
            # rbq_sim_* node names let gazebo.bash pkill only this launch's leftovers.
            package="robot_state_publisher", executable="robot_state_publisher", name="rbq_sim_rsp",
            output="screen",
            parameters=[{"robot_description": robot_description}, {"use_sim_time": True}],
        ),

        # base_link hub: identity to the URDF root
        static_tf("base", "0", "0", "0", "0", "0", "0", "trunk"),
        # LiDAR frames come from the URDF now (own link + fixed joint), so robot_state_publisher
        # owns trunk -> lidar_*. Publishing them here too would give those frames two parents.

        # Camera optical frames (base_link -> sensor)
        static_tf("bt0", "0.364000", "0.000000", "-0.024919", "0.000000", "2.792527", "0.000000", "bt0"),
        static_tf("bt1", "0.260970", "0.000000", "-0.045820", "0.000000", "-2.897247", "0.000000", "bt1"),
        static_tf("bt2", "-0.195150", "0.006500", "-0.046500", "0.000000", "3.141593", "0.000000", "bt2"),
        static_tf("bt3", "-0.352082", "-0.000011", "-0.018938", "3.020240", "2.621397", "-0.060599", "bt3"),
        static_tf("ft0", "0.388031", "-0.037500", "0.037764", "-1.570796", "0.000000", "-1.570796", "ft0"),
        static_tf("rr0", "-0.388031", "0.037500", "0.037764", "1.570796", "0.000000", "-1.570796", "rr0"),

        # RViz
        Node(
            package="rviz2", executable="rviz2", name="rbq_sim_rviz",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": True}],
            condition=rviz_on,
        ),
    ])
