import os
import sys
sys.path.append(os.path.join(os.getcwd(), 'UWRT_Controller_StateMachine', 'py_odrive', 'py_odrive'))
import can
import json
import datetime
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lib.utils import CanDevice, ProcessYaml
from lib.odrive_wrapper import OdriveEncode


class OdriveMsgSubscriber(Node):

    def __init__(self):
        super().__init__('OdriveManager')
        # Ros Messenger setup
        self.json_publisher_ = self.create_publisher(
            String,
            'OdriveJsonPub',
            10  # QoS history depth
        )
        self.json_subscription_ = self.create_subscription(
            String,
            'OdriveJsonSub',
            self.json_callback,
            10  # QoS history depth
        )
        self.can_setup()
        self.db = OdriveEncode(os.path.join(os.getcwd(), 'UWRT_Controller_StateMachine', 'py_odrive', 'config/odrive-cansimple.dbc'), use_jointconfig = False)
        self.json_subscription_

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
            except can.CanError:
                # Optionally log the error:
                # logging.error(f"Error setting up device '{description}'", exc_info=True)
                device = CanDevice(None)
            return device
        # Prepare containers for device instances and their mappings.
        self.device_instance = {}
        for description in self.yaml_dct.get_config().keys():
            device = setup_device(description)
            self.device_instance[description] = device
        # Output for debugging; consider using logging in production.
        print('Device Instances:', self.device_instance)
            
    def json_callback(self, msg):
        json_msg = json.loads(msg.data)
        can_msg = []
        pending = False
        if 'Type' in json_msg:
            if json_msg['Type'] != 'request':
                raise RuntimeError(f'Expected working directory to be request')
            if 'Command' in json_msg:
                if json_msg['Command'] == 'Set_Axis_State':
                    for axis_id, command in json_msg['Payload'].items():
                        can_msg.append(self.db.Set_Axis_State(int(axis_id), command))
                        if command == 'FULL_CALIBRATION_SEQUENCE':
                            pending = True
                elif json_msg['Command'] == 'Set_Controller_Mode':
                    for axis_id, command in json_msg['Payload'].items():
                        can_msg.append(self.db.Set_Controller_Mode(int(axis_id), command['control_mode'], command['input_mode']))
                elif json_msg['Command'] == 'Set_Input_Vel':
                    for axis_id, command in json_msg['Payload'].items():
                        can_msg.append(self.db.Set_Input_Vel(int(axis_id), command, 0, 0))
                if 'Target' in json_msg:
                    bus = self.device_instance[json_msg['Target']].get_bus()
        print(bus)
        for buf in can_msg:
            if bus is not None:
                bus.send(buf)
                print(buf)
        if pending == True:
            time.sleep(20)
        self.response_pub(json_msg['Stage'], json_msg['Target'], 'Success')
            
        
    def response_pub(self, stage, target, status):
        response = {'Stage': stage,
                    'Type': 'response',
                    'Target': target,
                    'Payload': status, 
                    'timestamp': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        msg = String()
        msg.data = json.dumps(response)
        self.json_publisher_.publish(msg)
        print(f'msg: {json.dumps(response)}')
        

def main():
    rclpy.init()
    node = OdriveMsgSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
