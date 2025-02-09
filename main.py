from py_odrive.lib.odrive_wrapper import OdriveEncode

test = OdriveEncode('/home/linyuchen/ODrive-Driver/config/odrive-cansimple.dbc', use_jointconfig = False)
print(test.Reboot(1))