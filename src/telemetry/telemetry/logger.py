import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix

import os

class LoggerToFile(Node):
    def __init__(self):
        super().__init__("LoggerToFile")

        self.imu_subscriber = self.create_subscription(
            Vector3,
            "/imu/euler",
            self.imu_cb,
            10
        )

        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_cb,
            10
        )

        self.p = os.path.join(os.getcwd(), "telemetry.csv")

    def imu_cb(self, msg):
        self.last_bearing = str(msg.z)

    def gps_cb(self, msg):
        self.get_logger().info("Writing to %s..." % self.p)
        self.get_logger().info(str(msg.latitude) + " " + str(msg.longitude) + " " +  self.last_bearing)

        with open(self.p, "a") as f:
            f.write("%s,%s,%s\n" % (str(msg.latitude), str(msg.longitude), self.last_bearing))


def main(args=None):
    rclpy.init(args=args)
    n = LoggerToFile()
    
    rclpy.spin(n)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
