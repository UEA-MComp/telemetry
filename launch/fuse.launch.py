import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")

def generate_launch_description():

    rtk_tf2_node = actions.Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        output = "screen",
        name = "gps_transform",
        arguments = ["-0.11", "0", "0.10", "0", "0", "0", "base_link", "gps_link"],
        parameters = [
            {'use_sim_time': True},
        ]
    )

    imu_tf2_node = actions.Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        output = "screen",
        name = "imu_transform",
        arguments = ["7", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        parameters = [
            {'use_sim_time': True},
        ]
    )

    navsat_transform_node = actions.Node(
        package = "robot_localization",
        executable = "navsat_transform_node",
        name = "navsat_transform_node",
        output = "screen",
        parameters = [
            {
                'use_sim_time': True,
                "magnetic_declination_radians": 4.71238,
                "zero_altitude": True,
                "yaw_offset": 0.0,
                "use_odometry_yaw": False,
                "wait_for_datum": False,
                "publish_filtered_gps": True,
                "broadcast_utm_transform": False

            }
        ],
        remappings = [
            ("/imu/data", "/imu"),
            ("/gps/fix", "/fix"),
            ("/odometry/filtered", "/odometry/filtered")
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        ExecuteProcess(
            cmd = ["ros2", "bag", "play", os.path.join(os.path.dirname(__file__), "..", "bags", "chapelfield.bag"), "--clock", "1"],
            output = "screen"
        ),
        rtk_tf2_node,
        imu_tf2_node,
        navsat_transform_node,
        # ExecuteProcess(
        #     cmd = [
        #         "ros2", "topic", "pub", "-1", "/initialpose", "geometry_msgs/PoseWithCovarianceStamped", 
        #         '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 0.1, y: 0.0, z: 0.0}, orientation: {w: 0.1}}, } }'
        #     ],
        #     output = "screen"
        # )
    ])

def main(argv):
    ld = generate_launch_description()

    print("Starting introspection of launch description...\n")
    print(LaunchIntrospector().format_launch_description(ld))
    print("Starting launch of launch description...\n")
    
    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == "__main__":
    main(sys.argv)