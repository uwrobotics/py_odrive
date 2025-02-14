from uwrt_ros_msg.msg import OdriveCmd
import rclpy
from rclpy.node import Node

class OdriveMsgSubscriber(Node):

    def __init__(self):
        super().__init__('odrive_msg_subscriber')
        self.subscription = self.create_subscription(
            OdriveCmd,
            'OdriveCmd',
            self.odrive_cmd_callback,
            10  # QoS history depth
        )
        # Prevent unused variable warning.
        self.subscription

    def odrive_cmd_callback(self, msg):
        # Print the received message fields.
        print("Axis ID:", msg.axis_id)
        print("Command:", msg.cmd)
        print("Payload:", msg.payload)


def main():
    rclpy.init()
    node = OdriveMsgSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
