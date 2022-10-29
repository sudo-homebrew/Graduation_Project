# Constants for the 'drive' value as defined by the Rovio API
int8 STOP=0
int8 FORWARD=1
int8 BACKWARD=2 
int8 STRAIGHT_LEFT=3
int8 STRAIGHT_RIGHT=4
int8 ROTATE_LEFT=5
int8 ROTATE_RIGHT=6
int8 DIAGONAL_FORWARD_LEFT=7
int8 DIAGONAL_FORWARD_RIGHT=8
int8 DIAGONAL_BACKWARD_LEFT=9
int8 DIAGONAL_BACKWARD_RIGHT=10
int8 HEAD_UP=11
int8 HEAD_DOWN=12
int8 HEAD_MIDDLE=13
# 14, 15, and 16 are reserved values
int8 ROTATE_LEFT_20_DEG=17
int8 ROTATE_RIGHT_20_DEG=18
int8 MIN_DRIVE_VAL=0
int8 MAX_DRIVE_VAL=18

# Constants useful for the 'speed' value as defined by the Rovio API
int8 FASTEST=1 
int8 SLOWEST=10
int8 MIN_SPEED_VAL=1
int8 MAX_SPEED_VAL=10

# The manual drive message contains a 'drive' and 'speed' value as defined by the Rovio API
int8 drive
int8 speed
