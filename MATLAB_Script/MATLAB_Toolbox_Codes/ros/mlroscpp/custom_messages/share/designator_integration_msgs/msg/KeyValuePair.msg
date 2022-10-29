# Purely for (de-)serialization purposes
int32 id
int32 parent

# Data
int32 TYPE_STRING=0
int32 TYPE_FLOAT=1
int32 TYPE_DATA=2
int32 TYPE_LIST=3
int32 TYPE_POSESTAMPED=4
int32 TYPE_POSE=5
int32 TYPE_DESIGNATOR_ACTION=6
int32 TYPE_DESIGNATOR_OBJECT=7
int32 TYPE_DESIGNATOR_LOCATION=8
int32 TYPE_DESIGNATOR_HUMAN=9
int32 TYPE_POINT=10
int32 TYPE_WRENCH=11
int32 TYPE_MATRIX=12
int32 TYPE_VECTOR=13

int32 type

string key
string value_string
float64 value_float
char[] value_data
float64[] value_array
geometry_msgs/PoseStamped value_posestamped
geometry_msgs/Pose value_pose
geometry_msgs/Point value_point
geometry_msgs/Wrench value_wrench
