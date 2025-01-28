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
        self.devices_config = utils.ProcessYaml(config_file)
        device_dct = self.devices_config.get_config()
        self.buses = {}
        self.buses_config = {}
        for device in device_dct:
            try:
                self.buses[device] =  utils.CanDevice(can.interface.Bus(bustype=self.devices_config.get_config(key='bus_type',device_name=device),
                                                                        channel=self.devices_config.get_config(key='channel',device_name=device),
                                                                        bitrate=self.devices_config.get_config(key='bitrate',device_name=device)), 'Online')
            # Associate device mapping
                self.buses_config[device] = self.devices_config.get_config(key='mapping',device_name=device)
            except OSError:
                self.buses[device] =  utils.CanDevice(None , 'offline')
                
    def get_device(self):
        pass

    def transmit(self, device, msg):
        assert self.buses[device].get_status(), f'can device {self.buses[device]} is not online at the moment'
        assert msg.msg_ready, f'Message to {device} is not prepared'
        self.buses[device].send(msg.buf)
        
    def recv(self):
        for bus in self.buses:
            self._recv(bus):
                
        
    # async listen private method
    def _recv(self, can_bus):
        can_bus.recv()
        
    
    
    
    
