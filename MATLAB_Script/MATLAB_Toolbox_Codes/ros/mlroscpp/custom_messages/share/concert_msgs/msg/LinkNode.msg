# Representation of a concert client node in the implementation graph

# unique identifier
string id

# tuple representing the client node - platform.system.robot.app, e.g. linux.ros.turtlebot.turtle_stroll
string tuple

# Constraints on how many of these nodes may exist
#   min, max not set -> min = 1, max = 1
#   min not set -> min = 1
#   max not set -> max = UNLIMITED_RESOURCE
# Zero is a valid minimum value
int8 UNLIMITED_RESOURCE=-1
int8 min
int8 max

# Force matching of robot name (only really useful for demos)
bool force_name_matching