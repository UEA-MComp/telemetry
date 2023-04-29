import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions
from launch.actions import SetEnvironmentVariable

NAVSAT_CONF_PATH = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "nmea_tcpclient_driver.yaml")

def generate_launch_description():
    navsat_node = actions.Node(
        package = "nmea_navsat_driver",
        executable = "nmea_tcpclient_driver",
        output = "screen",
        parameters = [
            NAVSAT_CONF_PATH
        ],
        remappings = [
            ("/fix", "/gps/fix"),
            ("/heading", "/gps/heading"),
            ("/vel", "/gps/vel"),
            ("/time_reference", "/gps/time_reference")
        ]
    )

    telemetry_node = actions.Node(
        package = "telemetry",
        executable = "telemetry",
        output = "screen"
    )

    imu_node = actions.Node(
        package = "bno085",
        executable = "bno085_publisher",
        output = "screen",
        remappings = [
            ("/IMU_Data", "/imu/data"),
            ("/Robot_Euler_Orientation", "/imu/euler")
        ]
    )
    
    rtk_tf2_node = actions.Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0.05", "0", "0", "0", "0", "0", "base_link", "rtk_link"]
    )


    # we could launch the rtkrcv tmux container here, with an ExecuteProcess(), e.g.
    #   ExecuteProcess(cmd=cmd, additional_env=env, output='screen')
    # currently not doing this because the rtk takes a while to warm up so its nice
    # to have it have a fix before we even start
    return LaunchDescription([
        # is there a way of setting different nodes to run in different domain IDs?
        # if so, it would be ideal to make telemetry_node run in domain ID 142 with a 
        # discovery server 10.13.13.4:11811 (the server VPN IP) so it can send telemetry
        # to the server in its own isolated thing
        #SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value='10.13.13.4:11811'),
        #SetEnvironmentVariable(name="ROS2_DOMAIN_ID", value="142"),
        navsat_node,
        telemetry_node,
        imu_node,
        rtk_tf2_node,
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

