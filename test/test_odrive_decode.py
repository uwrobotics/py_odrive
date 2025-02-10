import pytest
from py_odrive.lib.odrive_wrapper import OdriveDecode
import can
import cantools

@pytest.fixture
def read_dbc():
    return cantools.database.load_file('config/odrive-cansimple.dbc')

@pytest.fixture
def decode_instance():
    return OdriveDecode('config/odrive-cansimple.dbc', use_jointconfig=False)

@pytest.fixture
def axis_id():
    return 1

@pytest.fixture
def Heartbeat(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Heartbeat')
    data = msg.encode({'Axis_Error': 0, 'Axis_State': 'FULL_CALIBRATION_SEQUENCE', 'Motor_Error_Flag': 0, 'Encoder_Error_Flag': 0, 'Controller_Error_Flag': 0, 'Trajectory_Done_Flag': 0})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Heartbeat(decode_instance, axis_id, Heartbeat):
    cmd, result = decode_instance.decode_message(Heartbeat)
    assert cmd == 'Heartbeat'
    assert result == {str(axis_id): None}
    assert decode_instance.get_axis_state() == {str(axis_id): 'FULL_CALIBRATION_SEQUENCE'}

@pytest.fixture
def Heartbeat_Axis_Error(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Heartbeat')
    data = msg.encode({'Axis_Error': 'MOTOR_FAILED', 'Axis_State': 'FULL_CALIBRATION_SEQUENCE', 'Motor_Error_Flag': 0, 'Encoder_Error_Flag': 0, 'Controller_Error_Flag': 0, 'Trajectory_Done_Flag': 0})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Heartbeat_Axis_Error(decode_instance, axis_id, Heartbeat_Axis_Error):
    cmd, result = decode_instance.decode_message(Heartbeat_Axis_Error)
    assert cmd == 'Heartbeat'
    assert result == {str(axis_id): {'Axis_Error': 'MOTOR_FAILED', 'Axis_State': 'FULL_CALIBRATION_SEQUENCE', 'Motor_Error_Flag': 0, 'Encoder_Error_Flag': 0, 'Controller_Error_Flag': 0, 'Trajectory_Done_Flag': 0}}
    assert decode_instance.get_axis_state() == {str(axis_id): 'FULL_CALIBRATION_SEQUENCE'}
    
@pytest.fixture
def Heartbeat_Controller_Error(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Heartbeat')
    data = msg.encode({'Axis_Error': 0, 'Axis_State': 'FULL_CALIBRATION_SEQUENCE', 'Motor_Error_Flag': 0, 'Encoder_Error_Flag': 0, 'Controller_Error_Flag': 1, 'Trajectory_Done_Flag': 0})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Heartbeat_Controller_Error(decode_instance, axis_id, Heartbeat_Controller_Error):
    cmd, result = decode_instance.decode_message(Heartbeat_Controller_Error)
    assert cmd == 'Heartbeat'
    assert result == {str(axis_id): {'Axis_Error': 'NONE', 'Axis_State': 'FULL_CALIBRATION_SEQUENCE', 'Motor_Error_Flag': 0, 'Encoder_Error_Flag': 0, 'Controller_Error_Flag': 1, 'Trajectory_Done_Flag': 0}}
    assert decode_instance.get_axis_state() == {str(axis_id): 'FULL_CALIBRATION_SEQUENCE'}
    
@pytest.fixture
def Get_Encoder_Count(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Encoder_Count')
    data = msg.encode({'Count_in_CPR': 10, 'Shadow_Count': 20})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Get_Encoder_Count(decode_instance, axis_id, Get_Encoder_Count):
    cmd, result = decode_instance.decode_message(Get_Encoder_Count)
    assert cmd == 'Get_Encoder_Count'
    assert result == {str(axis_id): {'Count_in_CPR': 10, 'Shadow_Count': 20}}
    
@pytest.fixture
def Get_Motor_Error(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Motor_Error')
    data = msg.encode({'Motor_Error': 'PHASE_RESISTANCE_OUT_OF_RANGE'})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Get_Motor_Error(decode_instance, axis_id, Get_Motor_Error):
    cmd, result = decode_instance.decode_message(Get_Motor_Error)
    assert cmd == 'Get_Motor_Error'
    assert result == {str(axis_id): {'Motor_Error': 'PHASE_RESISTANCE_OUT_OF_RANGE'}}
    
@pytest.fixture
def Get_Encoder_Estimates(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Encoder_Estimates')
    data = msg.encode({'Vel_Estimate': 5, 'Pos_Estimate': 10})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Get_Encoder_Estimates(decode_instance, axis_id, Get_Encoder_Estimates):
    cmd, result = decode_instance.decode_message(Get_Encoder_Estimates)
    assert cmd == 'Get_Encoder_Estimates'
    assert result == {str(axis_id): None}
    assert decode_instance.get_encoder_est() == {str(axis_id): {'Vel_Estimate': 5, 'Pos_Estimate': 10}}