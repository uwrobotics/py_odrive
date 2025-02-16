import rclpy
from rclpy.node import Node

import datetime
import os
import can

from lib.utils import CanDevice, ProcessYaml

from uwrt_ros_msg.msg import OdriveCmd, MsgResponse


class OdriveMsgSubscriber(Node):

    def __init__(self):
        super().__init__('odrive_msg_subscriber')
        # Ros Messenger setup
        self.subscription = self.create_subscription(
            OdriveCmd,
            'OdriveCmd',
            self.odrive_cmd_callback,
            10  # QoS history depth
        )
        self.publisher_ = self.create_publisher(
            MsgResponse,
            'MsgResponse',
            10  # QoS history depth
        )
        self.subscription

    def can_setup(self):
        # Read Yaml Config
        yaml_dct = ProcessYaml('../config/config.yaml')
        dct_key = yaml_dct.get_config().keys()
        device_annotation = {}
        for description in dct_key:
            try:
                interface = yaml_dct.get_config(key='interface', device_name=description)
                channel = yaml_dct.get_config(key='channel', device_name=description)
                bitrate = yaml_dct.get_config(key='bitrate', device_name=description)
                bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
                device_annotation[description] = yaml_dct.get_config(key='mapping', device_name=description)
            except CanError:
                pass
            #log the result
    
    def odrive_cmd_callback(self, msg):
        # Print the received message fields.
        print("Axis ID:", msg.axis_id)
        print("Command:", msg.cmd)
        print("Payload:", msg.payload)

        status = True
        self.publish(status)
        
    def publish(self, status):
        msg = MsgResponse()
        msg.status = status
        msg.timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.publisher_.publish(msg)
        print(f'Response: {msg.status} timestamp: {msg.timestamp}')
        

def main():
    rclpy.init()
    node = OdriveMsgSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
