# This is a file in which the message publishing epos state is
# defined. It should return thesame types of inrmation as the Player
# driver.  
Header header        #standard header information

int64 raw_position   # position directly from readActualPosition
float64 position     # position in radians (I hope)

int64 raw_speed      # speed directly from readActualSpeed
float64 speed        # speed in radians (I hope)

float64 acceleration # computed acceleration
int16 current        # current directly from readActualCurrent

bool sync            # change to true if servo is not moving