import os
# ARM_SERIAL_PORT_DEFAULT = '/dev/ttyUSB0'  # replace with your default value
ARM_SERIAL_PORT_DEFAULT = os.getenv('ARM_SERIAL_PORT', '/dev/ttyUSB0')
