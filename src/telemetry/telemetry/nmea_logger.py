import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import datetime
import json
import pynmeagps
import dotenv
import base64
import os

import database



if not os.path.exists(".docker"):
    print("Not in docker... Using external database server...")
    import dotenv
    dotenv.load_dotenv(dotenv_path = "db.env")
    db_host = "192.168.1.5"
    LOGS_PATH = "/home/pi/logs" # no trailing slashes please
else:
    db_host = "db"
    LOGS_PATH = "/logs"

class LocationSubscriber(Node):

    def __init__(self, db: database.MowerDatabase):
        super().__init__('nmea_subscriber')
        self.db = db

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

            self.db.append_telemetry(
                json_["iqn"], 
                datetime.datetime.combine(datetime.date.today(), parsed_sentence.time),
                parsed_sentence.lat, 
                parsed_sentence.alt, parsed_sentence.lon
            )

        self.db.append_nmea_logfile(nmea_sentence, json_["iqn"], LOGS_PATH)
        


def main(args=None):
    rclpy.init(args=args)

    with database.MowerDatabase(host = db_host) as db:
        subscriber = LocationSubscriber(db)

        rclpy.spin(subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()