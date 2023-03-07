import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import pynmeagps
import base64


class LocationSubscriber(Node):

    def __init__(self):
        super().__init__('nmea_subscriber')
        self.subscription = self.create_subscription(
            String,
            'top_nmea',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        json_ = json.loads(msg.data)
        nmea_sentence = base64.b64decode(json_["nmea"])
        parsed_sentence = pynmeagps.NMEAReader.parse(nmea_sentence)
        # self.get_logger().info("Recieved NMEA message '%s' from '%s'" % (str(nmea_sentence), json_["iqn"]))
        if parsed_sentence.msgID == "GGA":
            print(parsed_sentence.lat, parsed_sentence.alt, parsed_sentence.lon, parsed_sentence.time)
        


def main(args=None):
    rclpy.init(args=args)

    subscriber = LocationSubscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()