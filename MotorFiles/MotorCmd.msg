# Motor commands

uint8 motor         # Motor index (1 to 6)
uint8 direction     # Direction: 1 (forward), 0 (backward)
uint8 speed         # Speed (0 to 255)

uint8 ALL = 255     # all motors or not

uint8 command_type  # command type: 0 (drive_motor), 1 (drive_all_motors), 2 (stop_motor), 3 (stop)

uint8 DRV_MOTOR = 0
uint8 DRV = 1
uint8 STOP_MOTOR = 2
uint8 STOP = 3
