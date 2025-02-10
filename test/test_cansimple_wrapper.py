import pytest
import os
from py_odrive.lib.cansimple_wrapper import CanWrapperEncode
import cantools
import can

@pytest.fixture
def test_dbc_path():
    return os.path.join('test/test_asset/odrive-cansimple.dbc')

@pytest.fixture
def cmd_set_axis_node_id():
    db = cantools.database.load_file(os.path.join('test/test_asset/odrive-cansimple.dbc'))
    return  db.get_message_by_name('Set_Axis_Node_ID')

@pytest.fixture
def axis_id():
    return 1

@pytest.fixture
def reboot(axis_id):
    db = cantools.database.load_file(os.path.join('test/test_asset/odrive-cansimple.dbc'))
    msg = db.get_message_by_name('Reboot')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False)

@pytest.fixture
def set_axis_state(axis_id):
    db = cantools.database.load_file(os.path.join('test/test_asset/odrive-cansimple.dbc'))
    msg = db.get_message_by_name('Set_Axis_State')
    data = msg.encode({'Axis_Requested_State': 0x03})
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, data=data)

@pytest.fixture
def get_encoder_estimate(axis_id):
    db = cantools.database.load_file(os.path.join('test/test_asset/odrive-cansimple.dbc'))
    msg = db.get_message_by_name('Get_Encoder_Estimates')
    return can.Message(arbitration_id=axis_id << 5 | msg.frame_id, is_extended_id=False, is_remote_frame = True)

def test_payload_encode(test_dbc_path, cmd_set_axis_node_id):
    db = CanWrapperEncode(test_dbc_path)
    result = db._encode_payload(cmd_set_axis_node_id, {'Axis_Node_ID': 10})
    assert result == {'Axis_Node_ID': 10}
    
def test_missing_payload(test_dbc_path, cmd_set_axis_node_id):
    with pytest.raises(Exception) as e_info:
        db = CanWrapperEncode(test_dbc_path)
        result = db._encode_payload(cmd_set_axis_node_id, {'a': 5})
        
def test_excess_payload(test_dbc_path, cmd_set_axis_node_id):
    db = CanWrapperEncode(test_dbc_path)
    result = db._encode_payload(cmd_set_axis_node_id, {'Axis_Node_ID': 10, 'a': 10})
    assert result == {'Axis_Node_ID': 10}
    
def test_reboot(test_dbc_path, axis_id, reboot):
    db = CanWrapperEncode(test_dbc_path)
    result = db._encode(axis_id,'Reboot')
    assert result.arbitration_id == reboot.arbitration_id
    assert result.dlc == reboot.dlc
    assert result.is_remote_frame == reboot.is_remote_frame
    assert result.data == reboot.data
    
def test_set_axis_state(test_dbc_path, axis_id, set_axis_state):
    db = CanWrapperEncode(test_dbc_path)
    result = db._encode(axis_id,'Set_Axis_State', payload = {'Axis_Requested_State': 0x03})
    assert result.arbitration_id == set_axis_state.arbitration_id
    assert result.dlc == set_axis_state.dlc
    assert result.is_remote_frame == set_axis_state.is_remote_frame
    assert result.data == set_axis_state.data
    
def test_get_encoder_estimate(test_dbc_path, axis_id, get_encoder_estimate):
    db = CanWrapperEncode(test_dbc_path)
    result = db._encode(axis_id,'Get_Encoder_Estimates', rtr = True)
    assert result.arbitration_id == get_encoder_estimate.arbitration_id
    assert result.dlc == get_encoder_estimate.dlc
    assert result.is_remote_frame == get_encoder_estimate.is_remote_frame
    assert result.data == get_encoder_estimate.data