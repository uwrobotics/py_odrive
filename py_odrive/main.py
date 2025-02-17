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
        can_setup()
        self.subscription

    def can_setup(self):
        # Read Yaml Config
        assert(os.getcwd() == '/home/uwrt/code_ws/py_odrive/py_odrive'), f'{os.getcwd()}'
        self.yaml_dct = ProcessYaml('./config/config.yaml')
        self.get_config = lambda key, description: self.yaml_dct.get_config(key=key, device_name=description)
        dct_key = self.yaml_dct.get_config().keys()
        self.device_instance = {}
        self.device_mapping = {}
        for description in dct_key:
            try:
                interface = self.get_config('interface', description)
                channel = self.get_config('channel', description)
                bitrate = self.get_config('bitrate', description)
                bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
                device = CanDevice(bus, status = 'online')
                self.device_instance[description] = device
                self.device_mapping[description] = self.yaml_dct.get_config(key='mapping', device_name=description)
            except can.CanError:
                device = CanDevice(None)
                self.device_instance[description] = device
                self.device_mapping[description] = None
            # Create result into CanInstance Object
    
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
