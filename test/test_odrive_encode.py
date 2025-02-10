import pytest
from py_odrive.lib.odrive_wrapper import OdriveEncode
import can
import cantools

@pytest.fixture
def read_dbc():
    return cantools.database.load_file('config/odrive-cansimple.dbc')

@pytest.fixture
def encode_instance():
    return OdriveEncode('config/odrive-cansimple.dbc', use_jointconfig=False)

@pytest.fixture
def axis_id():
    return 1

@pytest.fixture
def set_Estop(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Estop')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False)

def test_Estop(encode_instance, axis_id, set_Estop):
    result = encode_instance.set_Estop(axis_id)
    assert result.arbitration_id == set_Estop.arbitration_id
    assert result.dlc == set_Estop.dlc
    assert result.is_remote_frame == set_Estop.is_remote_frame
    assert result.data == set_Estop.data
    
@pytest.fixture
def Get_Encoder_Error(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Encoder_Error')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_et_Motor_Error(encode_instance, axis_id, Get_Encoder_Error):
    result = encode_instance.Get_Encoder_Error(axis_id)
    assert result.arbitration_id == Get_Encoder_Error.arbitration_id
    assert result.dlc == Get_Encoder_Error.dlc
    assert result.is_remote_frame == Get_Encoder_Error.is_remote_frame
    assert result.data == Get_Encoder_Error.data
    
@pytest.fixture
def Get_Motor_Error(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Motor_Error')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Motor_Error(encode_instance, axis_id, Get_Motor_Error):
    result = encode_instance.Get_Motor_Error(axis_id)
    assert result.arbitration_id == Get_Motor_Error.arbitration_id
    assert result.dlc == Get_Motor_Error.dlc
    assert result.is_remote_frame == Get_Motor_Error.is_remote_frame
    assert result.data == Get_Motor_Error.data
    
@pytest.fixture
def Get_Sensorless_Error(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Sensorless_Error')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Sensorless_Error(encode_instance, axis_id, Get_Sensorless_Error):
    result = encode_instance.Get_Sensorless_Error(axis_id)
    assert result.arbitration_id == Get_Sensorless_Error.arbitration_id
    assert result.dlc == Get_Sensorless_Error.dlc
    assert result.is_remote_frame == Get_Sensorless_Error.is_remote_frame
    assert result.data == Get_Sensorless_Error.data
    
@pytest.fixture
def Set_Axis_Node_ID(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Axis_Node_ID')
    data = msg.encode({'Axis_Node_ID': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Axis_Node_ID(encode_instance, axis_id, Set_Axis_Node_ID):
    result = encode_instance.Set_Axis_Node_ID(axis_id, 5)
    assert result.arbitration_id == Set_Axis_Node_ID.arbitration_id
    assert result.dlc == Set_Axis_Node_ID.dlc
    assert result.is_remote_frame == Set_Axis_Node_ID.is_remote_frame
    assert result.data == Set_Axis_Node_ID.data
    
@pytest.fixture
def Set_Axis_State_int(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Axis_State')
    data = msg.encode({'Axis_Requested_State': 3})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Axis_State_int(encode_instance, axis_id, Set_Axis_State_int):
    result = encode_instance.Set_Axis_State(axis_id, 3)
    assert result.arbitration_id == Set_Axis_State_int.arbitration_id
    assert result.dlc == Set_Axis_State_int.dlc
    assert result.is_remote_frame == Set_Axis_State_int.is_remote_frame
    assert result.data == Set_Axis_State_int.data
    
@pytest.fixture
def Set_Axis_State_str(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Axis_State')
    data = msg.encode({'Axis_Requested_State': 3})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Axis_State_str(encode_instance, axis_id, Set_Axis_State_str):
    result = encode_instance.Set_Axis_State(axis_id, 'FULL_CALIBRATION_SEQUENCE')
    assert result.arbitration_id == Set_Axis_State_str.arbitration_id
    assert result.dlc == Set_Axis_State_str.dlc
    assert result.is_remote_frame == Set_Axis_State_str.is_remote_frame
    assert result.data == Set_Axis_State_str.data
    
@pytest.fixture
def Get_Encoder_Estimates(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Encoder_Estimates')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Encoder_Estimates(encode_instance, axis_id, Get_Encoder_Estimates):
    result = encode_instance.Get_Encoder_Estimates(axis_id)
    assert result.arbitration_id == Get_Encoder_Estimates.arbitration_id
    assert result.dlc == Get_Encoder_Estimates.dlc
    assert result.is_remote_frame == Get_Encoder_Estimates.is_remote_frame
    assert result.data == Get_Encoder_Estimates.data
    
@pytest.fixture
def Get_Encoder_Count(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Encoder_Count')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Encoder_Count(encode_instance, axis_id, Get_Encoder_Count):
    result = encode_instance.Get_Encoder_Count(axis_id)
    assert result.arbitration_id == Get_Encoder_Count.arbitration_id
    assert result.dlc == Get_Encoder_Count.dlc
    assert result.is_remote_frame == Get_Encoder_Count.is_remote_frame
    assert result.data == Get_Encoder_Count.data

@pytest.fixture
def Set_Controller_Mode(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Controller_Mode')
    data = msg.encode({'Control_Mode': 'VELOCITY_CONTROL', 'Input_Mode': 'VEL_RAMP'})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Controller_Mode_int(encode_instance, axis_id, Set_Controller_Mode):
    result = encode_instance.Set_Controller_Mode(axis_id, 'VELOCITY_CONTROL', 'VEL_RAMP')
    assert result.arbitration_id == Set_Controller_Mode.arbitration_id
    assert result.dlc == Set_Controller_Mode.dlc
    assert result.is_remote_frame == Set_Controller_Mode.is_remote_frame
    assert result.data == Set_Controller_Mode.data

@pytest.fixture
def Set_Input_Pos(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Input_Pos')
    data = msg.encode({'Input_Pos': 15, 'Vel_FF': 10, 'Torque_FF': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Input_Pos(encode_instance, axis_id, Set_Input_Pos):
    result = encode_instance.Set_Input_Pos(axis_id, 15, 10, 5)
    assert result.arbitration_id == Set_Input_Pos.arbitration_id
    assert result.dlc == Set_Input_Pos.dlc
    assert result.is_remote_frame == Set_Input_Pos.is_remote_frame
    assert result.data == Set_Input_Pos.data
    
@pytest.fixture
def Set_Input_Vel(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Input_Vel')
    data = msg.encode({'Input_Vel': 10, 'Input_Torque_FF': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Input_Vel(encode_instance, axis_id, Set_Input_Vel):
    result = encode_instance.Set_Input_Vel(axis_id, 10, 5)
    assert result.arbitration_id == Set_Input_Vel.arbitration_id
    assert result.dlc == Set_Input_Vel.dlc
    assert result.is_remote_frame == Set_Input_Vel.is_remote_frame
    assert result.data == Set_Input_Vel.data
    
@pytest.fixture
def Set_Input_Torque(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Input_Torque')
    data = msg.encode({'Input_Torque': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Input_Torque(encode_instance, axis_id, Set_Input_Torque):
    result = encode_instance.Set_Input_Torque(axis_id, 5)
    assert result.arbitration_id == Set_Input_Torque.arbitration_id
    assert result.dlc == Set_Input_Torque.dlc
    assert result.is_remote_frame == Set_Input_Torque.is_remote_frame
    assert result.data == Set_Input_Torque.data
    
@pytest.fixture
def Set_Limits(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Limits')
    data = msg.encode({'Velocity_Limit': 5, 'Current_Limit': 10})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Limits(encode_instance, axis_id, Set_Limits):
    result = encode_instance.Set_Limits(axis_id, 5, 10)
    assert result.arbitration_id == Set_Limits.arbitration_id
    assert result.dlc == Set_Limits.dlc
    assert result.is_remote_frame == Set_Limits.is_remote_frame
    assert result.data == Set_Limits.data
    
@pytest.fixture
def Start_Anticogging(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Start_Anticogging')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False)

def test_Start_Anticogging(encode_instance, axis_id, Start_Anticogging):
    result = encode_instance.Start_Anticogging(axis_id)
    assert result.arbitration_id == Start_Anticogging.arbitration_id
    assert result.dlc == Start_Anticogging.dlc
    assert result.is_remote_frame == Start_Anticogging.is_remote_frame
    assert result.data == Start_Anticogging.data
    
@pytest.fixture
def Set_Traj_Vel_Limit(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Traj_Vel_Limit')
    data = msg.encode({'Traj_Vel_Limit': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Traj_Vel_Limit(encode_instance, axis_id, Set_Traj_Vel_Limit):
    result = encode_instance.Set_Traj_Vel_Limit(axis_id, 5)
    assert result.arbitration_id == Set_Traj_Vel_Limit.arbitration_id
    assert result.dlc == Set_Traj_Vel_Limit.dlc
    assert result.is_remote_frame == Set_Traj_Vel_Limit.is_remote_frame
    assert result.data == Set_Traj_Vel_Limit.data
    
@pytest.fixture
def Set_Traj_Accel_Limits(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Traj_Accel_Limits')
    data = msg.encode({'Traj_Accel_Limit': 5, 'Traj_Decel_Limit': 10})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Traj_Accel_Limits(encode_instance, axis_id, Set_Traj_Accel_Limits):
    result = encode_instance.Set_Traj_Accel_Limits(axis_id, 5, 10)
    assert result.arbitration_id == Set_Traj_Accel_Limits.arbitration_id
    assert result.dlc == Set_Traj_Accel_Limits.dlc
    assert result.is_remote_frame == Set_Traj_Accel_Limits.is_remote_frame
    assert result.data == Set_Traj_Accel_Limits.data
    
@pytest.fixture
def Set_Traj_Inertia(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Traj_Inertia')
    data = msg.encode({'Traj_Inertia': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Traj_Inertia(encode_instance, axis_id, Set_Traj_Inertia):
    result = encode_instance.Set_Traj_Inertia(axis_id, 5)
    assert result.arbitration_id == Set_Traj_Inertia.arbitration_id
    assert result.dlc == Set_Traj_Inertia.dlc
    assert result.is_remote_frame == Set_Traj_Inertia.is_remote_frame
    assert result.data == Set_Traj_Inertia.data
    
@pytest.fixture
def Get_Iq(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Iq')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Iq(encode_instance, axis_id, Get_Iq):
    result = encode_instance.Get_Iq(axis_id)
    assert result.arbitration_id == Get_Iq.arbitration_id
    assert result.dlc == Get_Iq.dlc
    assert result.is_remote_frame == Get_Iq.is_remote_frame
    assert result.data == Get_Iq.data
    
@pytest.fixture
def Get_Sensorless_Estimates(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Sensorless_Estimates')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Sensorless_Estimates (encode_instance, axis_id, Get_Sensorless_Estimates):
    result = encode_instance.Get_Sensorless_Estimates(axis_id)
    assert result.arbitration_id == Get_Sensorless_Estimates.arbitration_id
    assert result.dlc == Get_Sensorless_Estimates.dlc
    assert result.is_remote_frame == Get_Sensorless_Estimates.is_remote_frame
    assert result.data == Get_Sensorless_Estimates.data
    
@pytest.fixture
def Reboot(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Reboot')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False)

def test_Reboot(encode_instance, axis_id, Reboot):
    result = encode_instance.Reboot(axis_id)
    assert result.arbitration_id == Reboot.arbitration_id
    assert result.dlc == Reboot.dlc
    assert result.is_remote_frame == Reboot.is_remote_frame
    assert result.data == Reboot.data
    
@pytest.fixture
def Get_Bus_Voltage_Current(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Bus_Voltage_Current')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Bus_Voltage_Current (encode_instance, axis_id, Get_Bus_Voltage_Current):
    result = encode_instance.Get_Bus_Voltage_Current(axis_id)
    assert result.arbitration_id == Get_Bus_Voltage_Current.arbitration_id
    assert result.dlc == Get_Bus_Voltage_Current.dlc
    assert result.is_remote_frame == Get_Bus_Voltage_Current.is_remote_frame
    assert result.data == Get_Bus_Voltage_Current.data
    
@pytest.fixture
def Clear_Errors(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Clear_Errors')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False)

def test_Clear_Errors(encode_instance, axis_id, Clear_Errors):
    result = encode_instance.Clear_Errors(axis_id)
    assert result.arbitration_id == Clear_Errors.arbitration_id
    assert result.dlc == Clear_Errors.dlc
    assert result.is_remote_frame == Clear_Errors.is_remote_frame
    assert result.data == Clear_Errors.data
    
@pytest.fixture
def Set_Linear_Count(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Linear_Count')
    data = msg.encode({'Position': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Linear_Count(encode_instance, axis_id, Set_Linear_Count):
    result = encode_instance.Set_Linear_Count(axis_id, 5)
    assert result.arbitration_id == Set_Linear_Count.arbitration_id
    assert result.dlc == Set_Linear_Count.dlc
    assert result.is_remote_frame == Set_Linear_Count.is_remote_frame
    assert result.data == Set_Linear_Count.data
    
@pytest.fixture
def Set_Pos_Gain(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Pos_Gain')
    data = msg.encode({'Pos_Gain': 5})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Pos_Gain(encode_instance, axis_id, Set_Pos_Gain):
    result = encode_instance.Set_Pos_Gain(axis_id, 5)
    assert result.arbitration_id == Set_Pos_Gain.arbitration_id
    assert result.dlc == Set_Pos_Gain.dlc
    assert result.is_remote_frame == Set_Pos_Gain.is_remote_frame
    assert result.data == Set_Pos_Gain.data
    
@pytest.fixture
def Set_Vel_Gains(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Set_Vel_Gains')
    data = msg.encode({'Vel_Gain': 5, 'Vel_Integrator_Gain': 10})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

def test_Set_Vel_Gains(encode_instance, axis_id, Set_Vel_Gains):
    result = encode_instance.Set_Vel_Gains(axis_id, 5, 10)
    assert result.arbitration_id == Set_Vel_Gains.arbitration_id
    assert result.dlc == Set_Vel_Gains.dlc
    assert result.is_remote_frame == Set_Vel_Gains.is_remote_frame
    assert result.data == Set_Vel_Gains.data
    
@pytest.fixture
def Get_ADC_Voltage(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_ADC_Voltage')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_ADC_Voltage (encode_instance, axis_id, Get_ADC_Voltage):
    result = encode_instance.Get_ADC_Voltage(axis_id)
    assert result.arbitration_id == Get_ADC_Voltage.arbitration_id
    assert result.dlc == Get_ADC_Voltage.dlc
    assert result.is_remote_frame == Get_ADC_Voltage.is_remote_frame
    assert result.data == Get_ADC_Voltage.data
    
@pytest.fixture
def Get_Controller_Error(axis_id, read_dbc):
    msg = read_dbc.get_message_by_name('Get_Controller_Error')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame=True)

def test_Get_Controller_Error (encode_instance, axis_id, Get_Controller_Error):
    result = encode_instance.Get_Controller_Error(axis_id)
    assert result.arbitration_id == Get_Controller_Error.arbitration_id
    assert result.dlc == Get_Controller_Error.dlc
    assert result.is_remote_frame == Get_Controller_Error.is_remote_frame
    assert result.data == Get_Controller_Error.data