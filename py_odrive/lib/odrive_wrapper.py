from cansimple_wrapper import CanWrapperEncode

class OdriveEncode(CanWrapperEncode):
    def __init__(self):
        # init all axises here and it just need to search?
        self.axis = {}

    def set_estop(self, joint):
        axis_id = self.joint2axis(joint)
        return self._encode(axis_id, 'Estop')
        
    @staticmethod
    def joint2axis(joint):
        # convert input str joint into axises number reading the yaml dct
        pass