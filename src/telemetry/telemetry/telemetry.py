import rclpy
from rclpy.node import Node
import random
import json

from std_msgs.msg import String
import dotenv
import socket
import pynmeagps
import base64

NMEA_HOST = "localhost"
NMEA_PORT = 2121


class Telemetry(Node):

    def __init__(self):
        super().__init__('top_nmea')
        self.publisher_ = self.create_publisher(String, 'top_nmea', 10)

        self.stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.stream.connect((NMEA_HOST, NMEA_PORT))

        for raw_data, parsed_data in pynmeagps.NMEAReader(self.stream).iterate():
            json_ = {
                "iqn": self.get_uuid(),
                "nmea": base64.b64encode(raw_data).decode('ascii')
            }
            # encode bytes as base64 for JSON serialization

            msg = String()
            msg.data = json.dumps(json_)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)


    def get_uuid(self):
        iscsi_conf = dotenv.dotenv_values("/etc/iscsi/initiatorname.iscsi")
        return iscsi_conf["InitiatorName"]


def main(args=None):
    rclpy.init(args=args)
    p = Telemetry()
    rclpy.spin(p)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()