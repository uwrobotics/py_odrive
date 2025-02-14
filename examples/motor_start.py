import can
import time
import sys

# Setting import path
sys.path.append('../py_odrive')
from py_odrive.lib.odrive_wrapper import OdriveEncode
from py_odrive.lib.utils import ProcessYaml, CanDevice


config = ProcessYaml('config/config.yaml')
interface = config.get_config(key='interface', device_name='Drivetrain')
channel = config.get_config(key='channel', device_name='Drivetrain')
bitrate = config.get_config(key='bitrate', device_name='Drivetrain')

bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
db = OdriveEncode('config/odrive-cansimple.dbc', use_jointconfig = False)
msg = db.Set_Axis_State(2, 'FULL_CALIBRATION_SEQUENCE')
bus.send(msg)
time.sleep(30)
msg = db.Set_Axis_State(2, 'CLOSED_LOOP_CONTROL')
bus.send(msg)
msg = db.Set_Input_Vel(2,0,0)
print(msg)
bus.send(msg)