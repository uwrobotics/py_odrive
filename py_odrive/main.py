import rclpy
from rclpy.node import Node

import datetime
import os
import sys
sys.path.append(os.path.join(os.getcwd(), 'UWRT_Controller_StateMachine', 'py_odrive', 'py_odrive'))
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
        self.status_publisher_ = self.create_publisher(
            MsgResponse,
            'MsgResponse',
            10  # QoS history depth
        )
        self.init_publisher_ = self.create_publisher(
            OdriveCmd,
            'DeviceInit',
            10  # QoS history depth
        )
        self.can_setup()
        self.subscription

    def can_setup(self):
        """Setup CAN devices using the configuration from a YAML file."""
        
        # Verify current working directory.
        expected_cwd = '/home/uwrt/ros_ws'
        current_cwd = os.getcwd()
        if current_cwd != expected_cwd:
            raise RuntimeError(f'Expected working directory to be {expected_cwd}, got {current_cwd}')
        
        # Load YAML configuration.
        config_path = os.path.join(current_cwd, 'UWRT_Controller_StateMachine', 'py_odrive', 'config', 'config.yaml')
        self.yaml_dct = ProcessYaml(config_path)
        
        def get_config(key, description):
            """Helper function to fetch configuration values."""
            return self.yaml_dct.get_config(key=key, device_name=description)
        
        def setup_device(description):
            """
            Setup a single CAN device given its description.
            Returns:
                device: CanDevice instance.
                mapping: Device mapping (or None on error).
            """
            try:
                interface = get_config('interface', description)
                channel = get_config('channel', description)
                bitrate = get_config('bitrate', description)
                bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
                device = CanDevice(bus, status='online')
                mapping = self.yaml_dct.get_config(key='mapping', device_name=description)
            except can.CanError:
                # Optionally log the error:
                # logging.error(f"Error setting up device '{description}'", exc_info=True)
                device = CanDevice(None)
                mapping = None
            return device, mapping

        # Prepare containers for device instances and their mappings.
        self.device_instance = {}
        self.device_mapping = {}

        for description in self.yaml_dct.get_config().keys():
            device, mapping = setup_device(description)
            self.device_instance[description] = device
            self.device_mapping[description] = mapping

        # Output for debugging; consider using logging in production.
        print('Device Instances:', self.device_instance)
        print('Device Mappings:', self.device_mapping)
    
    def odrive_cmd_callback(self, msg):
        # Print the received message fields.
        print("Description: ", msg.description)
        print("Axis ID:", msg.axis_id)
        print("Command:", msg.cmd)
        print("Payload:", msg.payload)
        if msg.description == 'None':
            self.init_publish()
        else:
            status = True
            self.status_publish(status)
    
    def init_publish(self):
        msg = OdriveCmd()
        msg.description = ','.join(self.device_instance.keys())
        axis_id = ''
        for key in self.device_instance.keys():
            axis_id += key
            axis_id += ':'
            axis_id += ','.join(self.device_mapping[key])
            axis_id += ';'
        msg.axis_id = axis_id
        msg.cmd = 'None'
        msg.payload = 'None'
        self.init_publisher_.publish(msg)
        print(f'description: {msg.description} axis_id: {msg.axis_id}')
        
    def status_publish(self, status):
        msg = MsgResponse()
        msg.status = status
        msg.timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.status_publisher_.publish(msg)
        print(f'Response: {msg.status} timestamp: {msg.timestamp}')
        

def main():
    rclpy.init()
    node = OdriveMsgSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
