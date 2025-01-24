import os
import multiprocessing

# look into workers
import can
import utils

# Final pr need to include proper unittesting and mocking
# Impl typing in the system
class OdriveV3:
    def __init__(self, config_file):
        # Todo:
        # - device lifetime (online, offline, not available, link retry)
        # - integrate LOG
        assert os.path.isfile(config_file), f"filepath:{config_file} does not exist"
        self.devices_config = utils.ProcessYaml(config_file).get_config()
        for device in self.devices_config:
            device_config = self.devices_config.get_result(device)
            try:
                self.buses[device] =  utils.CanDevice(can.interface.Bus(bustype=device_config['bus_type'],
                                                                        channel=device_config['channel'],
                                                                        bitrate=device_config['bitrate']), 'Online')
            # Associate device mapping
            except:
                self.buses[device] =  CanDevice(None , 'offline')

    def transmit(self, attribute, msg):
        assert self.buses[attribute].get_status(), f'can device {self.buses[attribute]} is not online at the moment'
        assert msg.msg_ready, f'Message to {attribute} is not prepared'
        self.buses[attribute].send(msg.buf)
        
    # async listen 
    def recv(self, can_bus):
        can_bus.recv()
        
    
    
    
    
