import can
import os

# look into workers
import multiprocessing

# include final test is a submitting pr

class CanDevice:
    def __init__(self, can_device, status):
        self.bus = can_device
        self.status = status
        
    def get_bus(self):
        return self.can_device
    
    def set_status(self, status):
        self.status = status
        
    def get_status(self):
        return self.status



class OdriveV3:
    def __init__(self, config_f):
        # Todo:
        # - device lifetime (online, offline, not available, link retry)
        # - integrate LOG
        assert os.path.isfile(config_f), f"filepath:{config_f} does not exist"
        self.can_config = self.read_config(config_f)
        for can_instance in self.can_config:
            self.buses[can_instance['attribute']] =  CanDevice(can.interface.Bus(bustype=can_instance['bus_type'],
                                                                        channel=can_instance['channel'],
                                                                        bitrate=can_instance['bitrate']), 'Online')

    def transmit(self, attribute, msg):
        assert self.buses[attribute].get_status(), f'can device {self.buses[attribute]} is not online at the moment'
        assert msg.msg_ready, f'Message to {attribute} is not prepared'
        self.buses[attribute].send(msg.buf)
        
    # what is the best way to handle async listen    
    def recv(self, attribute):
        pass
    
    # return lists of can device with properity
    def read_config(self, config_f):
        pass