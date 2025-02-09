import pytest
import os
import can
import cantools

@pytest.fixture
def dbc_db():
    return cantools.database.load_file("test/test_asset/odrive-cansimple.dbc")

@pytest.fixture
def AXIS_STATE_FULL_CALIBRATION_SEQUENCE():
    return b'\x03\x00\x00\x00\x00\x00\x00\x00'

@pytest.fixture
def AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD():
    return can.Message(timestamp=0.0, arbitration_id=0xc7, is_extended_id=False, dlc=8, data=[0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0])

@pytest.fixture
def AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD():
    return n

@pytest.fixture
def axisID():
    return 6

def test_encode_axis_calibration(dbc_db, axisID, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD):
    msg = dbc_db.get_message_by_name('Set_Axis_State')
    payload = msg.encode({'Axis_Requested_State': 0x03})
    assert payload == AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    msg = can.Message(arbitration_id=msg.frame_id | axisID << 5, is_extended_id=False, data=payload)
    assert msg.arbitration_id == AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.arbitration_id
    assert msg.is_extended_id == AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.is_extended_id
    assert msg.data == AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.data

def test_encode_command_axis_calibration(dbc_db, axisID, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD):
    msg = dbc_db.get_message_by_name('Set_Axis_State')
    payload = msg.encode({'Axis_Requested_State': 'FULL_CALIBRATION_SEQUENCE'})
    assert payload == AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    msg = can.Message(arbitration_id=msg.frame_id | axisID << 5, is_extended_id=False, data=payload)
    assert msg.arbitration_id == AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.arbitration_id
    assert msg.is_extended_id == AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.is_extended_id
    assert msg.data == AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.data
    
def test_decode_axis_calibration(dbc_db, axisID, AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD, AXIS_STATE_FULL_CALIBRATION_SEQUENCE):
    msg = dbc_db.get_message_by_name('Set_Axis_State')
    assert AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.arbitration_id == msg.frame_id | axisID << 5
    assert dbc_db.decode_message('Set_Axis_State', AXIS_STATE_FULL_CALIBRATION_SEQUENCE_CMD.data)['Axis_Requested_State'] == 'FULL_CALIBRATION_SEQUENCE'
    
def test_encode_set_limit()