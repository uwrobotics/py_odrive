import can
import cantools
from typing import Optional
import time

class CanWrapperEncode:
    '''
    Handles CAN message encoding for communication between user and device.
    
    Data Flow:
        - User2Device: Converts user-friendly commands to CAN messages.
        - Device2User: Converts received CAN messages to readable user format.
    '''
    def __init__(self, dbc_filepath: str):
        '''
        Initializes the encoder with the provided DBC file.
        
        :param dbc_filepath: Path to the DBC file.
        '''
        self.db = cantools.database.load_file(dbc_filepath)
        self.encode_arbitration_id = lambda axis_id, msg_id: (axis_id << 5) | msg_id
    
    def _encode(self, axis_id: int, cmd: str, payload: Optional[dict] = None, rtr: Optional[bool] = False) -> can.Message:
        '''
        Encodes a command into a CAN message.
        
        :param axis_id: ID of the target axis.
        :param cmd: Command name as defined in the DBC file.
        :param payload: Optional dictionary containing signal values.
        :param rtr: Boolean indicating if the message is a remote frame.
        :return: A CAN message ready for transmission.
        '''
        msg = self.db.get_message_by_name(cmd)
        data = {}
        
        if msg.length == 0:
            assert payload is None, "Payload must be None for zero-length messages."
        elif msg.receivers == {'Master'}:
            assert rtr is True, "RTR must be True for messages received by 'Master'."
        else:
            data = msg.encode(self._encode_payload(msg, payload))
        
        return can.Message(
            arbitration_id=self.encode_arbitration_id(axis_id, msg.frame_id),
            is_extended_id=False,
            is_remote_frame=rtr,
            data=data
        )
    
    def _encode_payload(self, msg, payload: dict) -> dict:
        '''
        Encodes the payload for a given message.
        
        :param msg: CAN message definition from the DBC file.
        :param payload: Dictionary containing signal values.
        :return: Encoded payload dictionary.
        '''
        dct = {}
        for signal in msg.signals:
            try:
                if signal.choices is not None:
                    assert payload[signal.name] in signal.choices.values() or payload[signal.name] in signal.choices, \
                        f"Invalid value for {signal.name}: {payload[signal.name]}"
                else:
                    assert isinstance(payload[signal.name], int), f"{signal.name} must be an integer."
                dct[signal.name] = payload[signal.name]
            except KeyError:
                raise ValueError(f"Missing required signal: {signal.name}")
        
        return dct
    

class CanWrapperDecode:
    '''
    Handles decoding of received CAN messages.
    '''
    def __init__(self, dbc_filepath: str):
        '''
        Initializes the decoder with the provided DBC file.
        
        :param dbc_filepath: Path to the DBC file.
        '''
        self.db = cantools.database.load_file(dbc_filepath)
        self.get_axis_id = lambda buf: buf.arbitration_id >> 5
        self.get_frame_id = lambda buf: buf.arbitration_id & 0b11111
        
    def decode_message(self, buf: can.Message):
        '''
        Decodes a received CAN message.
        
        :param buf: The received CAN message buffer.
        :return: Tuple containing axis ID, message definition, and decoded message data.
        '''
        axis_id = self.get_axis_id(buf)
        frame_id = self.get_frame_id(buf)
        
        return axis_id, self.db.get_message_by_frame_id(frame_id), self.db.decode_message(frame_id, buf.data)
