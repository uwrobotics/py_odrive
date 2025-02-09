from cansimple_wrapper import CanWrapperEncode

class OdriveEncode(CanWrapperEncode):
    def __init__(self):
        # init all axises here and it just need to search?
        self.axis = {}

    def set_estop(self, joint):
        axis_id = self.joint2axis(joint)
        return self._encode(axis_id, 'Estop')
    
    def get_Motor_Error(self, joint):
        axis_id = self.joint2axis(joint)
        return self._encode(axis_id, 'Get_Motor_Error', rtr=True)
        
    def get_Encoder_Error(self, joint):
        axis_id = self.joint2axis(joint)
        return self._encode(axis_id, 'Get_Encoder_Error', rtr=True)   
     
    def Get_Sensorless_Error(self, joint):
        axis_id = self.joint2axis(joint)
        return self._encode(axis_id, 'Get_Sensorless_Error', rtr=True)
    
    def Set_Axis_Node_ID(self, joint, id):
        '''Use this method with caution'''
        axis_id = self.joint2axis(joint)
        payload = {'Axis_Node_ID': id}
        return self._encode(axis_id, 'Get_Sensorless_Error', payload=payload)
        
    @staticmethod
    def joint2axis(joint):
        # convert input str joint into axises number reading the yaml dct
        pass