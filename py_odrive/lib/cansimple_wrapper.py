import can
import cantools
from typing import Optional
import time

class CanWrapperEncode:
    '''
    Data flow:
        User2Device: 
            User: axis_id, cmd_name, attribute && payload
            Device: axis_id, msg_id, buf
        Device2User:
            Device: axis_id, msg_id, buf
            User: axis_id, cmd_name, attribute && payload
    '''
    def __init__(self, dbc_filepath):
        self.db = cantools.database.load_file(dbc_filepath)
        self.encode_arbitration_id = lambda axis_id, msg_id: axis_id << 5 | msg_id
    
    def _encode(self, axis_id, cmd: str, payload: Optional[dict] = None):
        msg = self.db.get_message_by_name(cmd)
        if msg.length == 0:
            assert payload is None
        else:
            data = self._encode_payload(msg, payload)
        return can.Message(arbitration_id=self.encode_arbitration_id(axis_id, msg.frame_id), is_extended_id=False, data=data)
            
    def _encode_payload(self, msg, payload):
        dct = {}
        for signal in msg.signals:
            try:
                dct[signal.name] = payload[signal.name]
            except KeyError:
                dct[signal.name] = 0
                # log instance
        return msg.encode(dct)
    
    