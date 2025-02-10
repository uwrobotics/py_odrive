# ODrive-Driver

Todo:

- Decode function
- Main function demo
- Lifecycle
- ROS Message communication
- LCM communication
- investigate OBS live stream

# System Highlights:

## Lifecycle:

* on\_configure: calibration of direction testing

* on\_activate: configure control mode & action
* on\_deactivate: clear_error/ reboot

## Msg:

* Milestone 1: support velcity control and a subset of command (hardcoded format)
* Milestone 2: interactive argument -> user give the command name and can obtain the attribute fields
