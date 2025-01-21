import os
import multiprocessing

# look into workers
import can
import yaml

# Final pr need to include proper unittesting and mocking
# Impl typing in the system

class CanDevice:
    def __init__(self, can_device, 
                 status = 'offline'):
        self.bus = can_device
        self.toggle_status(status)
        
    def get_bus(self):
        return self.bus
    
    def toggle_status(self, status = 'offline'):
        if status == 'online':
            self.status = status
        else:
            self.status = 'offline'
        
    def get_status(self):
        return self.status



class ProcessYaml:
    def __init__(self, config_f):
        self.read_config(config_f)
        
    def get_result(self, attribute):
        if isinstance(self.content[attribute], list):
            return self._list2dict(self.content[attribute])
        else:
            return self.content[attribute]
    
    # return lists of can device with properity
    def read_config(self, config_f):
        file = open(config_f, 'r')
        self.yaml_obj = yaml.safe_load(file)
            
    # convert the list to dict type
    def _list2dict(self, yaml_list):
        dct = {}
        for sub_dct in yaml_list:
            dct.update(sub_dct)
        return dct



class OdriveV3:
    def __init__(self, config_f):
        # Todo:
        # - device lifetime (online, offline, not available, link retry)
        # - integrate LOG
        assert os.path.isfile(config_f), f"filepath:{config_f} does not exist"
        self.devices_config = ProcessYaml(config_f)
        for device in self.devices_config:
            device_config = self.devices_config.get_result(device)
            self.buses[device] =  CanDevice(can.interface.Bus(bustype=device_config['bus_type'],
                                                                        channel=device_config['channel'],
                                                                        bitrate=device_config['bitrate']), 'Online')
            # Associate device mapping

    def transmit(self, attribute, msg):
        assert self.buses[attribute].get_status(), f'can device {self.buses[attribute]} is not online at the moment'
        assert msg.msg_ready, f'Message to {attribute} is not prepared'
        self.buses[attribute].send(msg.buf)
        
    # what is the best way to handle async listen    
    def recv(self, attribute):
        pass
    
    
    
    
