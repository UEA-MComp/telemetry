import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import dotenv

class RTKPublisher(Node):

    topic_name = "rtk_top"
    queue_size = 10

    def __init__(self):
        super().__init__("rtk_pub")
        self.publisher_ = self.create_publisher(String, self.topic_name, self.queue_size)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = json.dumps({"iqn": self.get_uuid()})
        self.publisher_.publish(msg)

        self.i += 1

    def get_uuid(self):
        iscsi_conf = dotenv.dotenv_values("/etc/iscsi/initiatorname.iscsi")
        return iscsi_conf["InitiatorName"]
        # with open("/etc/iscsi/initiatorname.iscsi", "r") as f:
        #     return f.read()

def main(args = None):
    rclpy.init(args = args)
    rtk_publisher = RTKPublisher()
    rclpy.spin(rtk_publisher)

    rtk_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
