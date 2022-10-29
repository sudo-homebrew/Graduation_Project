# Send data to serial port.

# Eeither 0 (connectionless) or an unexpired socket identifier
# from an earlier communication.
uint32  socket

# Data to be sent.
uint8[] data

# Number of seconds to keep the port locked after this communication.
float32 timeout
