from .cansimple_wrapper import CanWrapperEncode, CanWrapperDecode
from typing import Optional

'''
Need unit testing
for sunday integration testing will write a main fn in example folder
this node need lifetime management
onconfigure(FULL_SEQUENCE_CALIBRATION)
during operation(Check the current mode -> send set vel command)
'''

class OdriveEncode(CanWrapperEncode):
    def __init__(self, dbc_filepath, use_jointconfig: Optional[bool] = True, 
                 mapping_config: Optional[dict] = {'FrontLeftMotor': 1,'MiddleLeftMotor': 2 ,'BackLeftMotor': 3, 'FrontRightMotor': 4, 'MiddleRightMotor': 5, 'BackRightMotor': 6}):
        # load all axis from config
        super().__init__(dbc_filepath)
        self.use_jointconfig = use_jointconfig
        '''support reading the config file'''
        if self.use_jointconfig == True:
            assert mapping_config is not None
            self.axis = mapping_config

    def set_Estop(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Estop')
    
    def Get_Motor_Error(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Motor_Error', rtr=True)
        
    def Get_Encoder_Error(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Encoder_Error', rtr=True)   
     
    def Get_Sensorless_Error(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Sensorless_Error', rtr=True)
    
    def Set_Axis_Node_ID(self, joint, id):
        '''Use this method with caution'''
        axis_id = self.joint2axis(joint)
        payload = {'Axis_Node_ID': id}
        return super()._encode(axis_id, 'Set_Axis_Node_ID', payload=payload)
    
    def Set_Axis_State(self, joint, state):
        axis_id = self.joint2axis(joint)
        payload = {'Axis_Requested_State': state}
        return super()._encode(axis_id, 'Set_Axis_State', payload=payload)
    
    def Get_Encoder_Estimates(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Encoder_Estimates', rtr=True)
    
    def Get_Encoder_Count(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Encoder_Count', rtr=True)
    
    def Set_Controller_Mode(self, joint, control_mode, input_mode):
        axis_id = self.joint2axis(joint)
        payload = {'Control_Mode': control_mode, 'Input_Mode': input_mode}
        return super()._encode(axis_id, 'Set_Controller_Mode', payload=payload)
    
    def Set_Input_Pos(self, joint, input_pos, vel_ff, torque_ff):
        axis_id = self.joint2axis(joint)
        payload = {'Input_Pos': input_pos, 'Vel_FF': vel_ff, 'Torque_FF': torque_ff}
        return super()._encode(axis_id, 'Set_Input_Pos', payload=payload)
    
    def Set_Input_Vel(self, joint, input_vel, input_torque_ff):
        axis_id = self.joint2axis(joint)
        payload = {'Input_Vel': input_vel, 'Input_Torque_FF': input_torque_ff}
        return super()._encode(axis_id, 'Set_Input_Vel', payload=payload)
    
    def Set_Input_Torque(self, joint, input_torque):
        axis_id = self.joint2axis(joint)
        payload = {'Input_Torque': input_torque}
        return super()._encode(axis_id, 'Set_Input_Torque', payload=payload)
    
    def Set_Limits(self, joint, velocity_limit, current_limit):
        axis_id = self.joint2axis(joint)
        payload = {'Velocity_Limit': velocity_limit, 'Current_Limit': current_limit}
        return super()._encode(axis_id, 'Set_Limits', payload=payload)
    
    def Start_Anticogging(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Start_Anticogging')
    
    def Set_Traj_Vel_Limit(self, joint, traj_vel_limit):
        axis_id = self.joint2axis(joint)
        payload = {'Traj_Vel_Limit': traj_vel_limit}
        return super()._encode(axis_id, 'Set_Traj_Vel_Limit', payload=payload)
    
    def Set_Traj_Accel_Limits(self, joint, traj_accel_limit, traj_decel_limit):
        axis_id = self.joint2axis(joint)
        payload = {'Traj_Accel_Limit': traj_accel_limit, 'Traj_Decel_Limit': traj_decel_limit}
        return super()._encode(axis_id, 'Set_Traj_Accel_Limits', payload=payload)
    
    def Set_Traj_Inertia(self, joint, traj_inertia):
        axis_id = self.joint2axis(joint)
        payload = {'Traj_Inertia': traj_inertia}
        return super()._encode(axis_id, 'Set_Traj_Inertia', payload=payload)
    
    def Get_Iq(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Iq', rtr=True)
    
    def Get_Sensorless_Estimates(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Sensorless_Estimates', rtr=True)
    
    def Reboot(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Reboot')
    
    def Get_Bus_Voltage_Current(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Bus_Voltage_Current', rtr=True)
    
    def Clear_Errors(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Clear_Errors')
    
    def Set_Linear_Count(self, joint, position):
        axis_id = self.joint2axis(joint)
        payload = {'Position': position}
        return super()._encode(axis_id, 'Set_Linear_Count', payload=payload)
    
    def Set_Pos_Gain(self, joint, pos_gain):
        axis_id = self.joint2axis(joint)
        payload = {'Pos_Gain': pos_gain}
        return super()._encode(axis_id, 'Set_Pos_Gain', payload=payload)
    
    def Set_Vel_Gains(self, joint, vel_gain, vel_integrator_gain):
        axis_id = self.joint2axis(joint)
        payload = {'Vel_Gain': vel_gain, 'Vel_Integrator_Gain': vel_integrator_gain}
        return super()._encode(axis_id, 'Set_Vel_Gains', payload=payload)
    
    def Get_ADC_Voltage(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_ADC_Voltage', rtr=True)
    
    def Get_Controller_Error(self, joint):
        axis_id = self.joint2axis(joint)
        return super()._encode(axis_id, 'Get_Controller_Error', rtr=True)
        
    def joint2axis(self, joint):
        # convert input str joint into axises number reading the yaml dct
        if self.use_jointconfig == True:
            return self.axis[joint]
        else:
            assert isinstance(joint, int)
            return joint


# can.Message(timestamp=0.0, arbitration_id=0x21, is_extended_id=False, dlc=8, data=[0x0, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0])
# can.Message(timestamp=0.0, arbitration_id=0x23, is_extended_id=False, dlc=8, data=[0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0])
class OdriveDecode(CanWrapperDecode):
    def __init__(self, dbc_filepath,  use_jointconfig: Optional[bool] = True,
                 mapping_config: Optional[dict] = {'FrontLeftMotor': 1,'MiddleLeftMotor': 2 ,'BackLeftMotor': 3, 'FrontRightMotor': 4, 'MiddleRightMotor': 5, 'BackRightMotor': 6}):
        super().__init__(dbc_filepath)
        self.use_jointconfig = use_jointconfig
        '''support reading the config file'''
        if self.use_jointconfig == True:
            assert mapping_config is not None
            self.axis = mapping_config
        self.axis_state = {}
        self.encoder_info = {'Vel_Estimate': 0, 'Pos_Estimate': 0}
            
    def get_axis_state(self):
        '''
        update every 5 sec
        '''
        return self.axis_state
    
    def get_encoder_est(self):
        '''
        impl graph plot?
        '''
        return self.encoder_info
            
    def axis2joint(self, axis):
        if self.use_jointconfig == True:
            name = [name for name, value in self.axis.items() if name == axis]
            assert len(name) == 1
            return name[0]
        else:
            assert isinstance(axis, int)
            return axis
        
    def decode_message(self, buf):
        axis_id, cmd, payload = super().decode_message(buf)
        joint = self.axis2joint(axis_id)
        if cmd.name == 'Heartbeat':
            msg = self.Heartbeat(joint, cmd, payload)
            if msg != None:
                return cmd.name, {str(joint): msg}
            else:
                return None
        elif cmd.name == 'Get_Encoder_Estimates':
            self.Get_Encoder_Estimates(payload)
            return None
        elif cmd.name in ['Get_Motor_Error', 'Get_Encoder_Error', 'Get_Sensorless_Error', 'Get_Encoder_Count', 'Get_Iq', 'Get_Sensorless_Estimates', 'Get_Bus_Voltage_Current', 'Get_ADC_Voltage', 'Get_Controller_Error']:
            return cmd.name, {str(joint): payload}
        else: 
            return None
            
    def Heartbeat(self, axis_id, cmd, payload):
        for signal in cmd.signals:
            if signal.name == 'Axis_State':
                self.axis_state[str(axis_id)] = payload['Axis_State']
            elif payload[signal.name] == 0:
                pass
            else:
                return payload
        return None
    
    def Get_Encoder_Estimates(self, payload):
        if 'Vel_Estimate' in payload:
            self.encoder_info['Vel_Estimate'] = payload['Vel_Estimate']
        else:
            raise ValueError
        if 'Pos_Estimate' in payload:
            self.encoder_info['Pos_Estimate'] = payload['Pos_Estimate']
        else:
            raise ValueError