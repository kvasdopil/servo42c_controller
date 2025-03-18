"""Constants used throughout the servo42c_controller package."""

# Motor configuration
STEPS_PER_REV = 200  # Base steps per revolution
MICROSTEP_FACTOR = 8  # Microstepping factor
GEAR_RATIO = 37      # Gear reduction ratio
PULSES_PER_ROTATION = STEPS_PER_REV * MICROSTEP_FACTOR * GEAR_RATIO

# Limits and safety
MIN_ANGLE = -360.0  # degrees
MAX_ANGLE = 360.0   # degrees
MIN_SPEED = 1
MAX_SPEED = 120
POSITION_TOLERANCE = 0.5  # degrees

# Serial communication
SERIAL_TIMEOUT = 1.0  # seconds
MAX_RECONNECT_ATTEMPTS = 3

# Command codes
GET_PULSES = 0x33
GET_SERIAL_ENABLED = 0xf3
ROTATE = 0xfd
STOP = 0xf7

# ROS2 settings
PUBLISHER_QOS = 10
STATUS_UPDATE_RATE = 0.1  # seconds 