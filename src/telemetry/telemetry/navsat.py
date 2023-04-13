import rclpy
from rclpy.node import Node
import random
import json

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class NavSatTranslation(Node):
    def __init__(self):
        super().__init__("NavSatTranslation")

        self.telemetry = self.create_subscription(String, "top_nmea", self.nmea_sentence_callback, 10)

    def nmea_sentence_callback(self, msg):
        