# 
# Describes an interactive concert client.
#
# Unique names - human consumable rocon name as well as globally unique name
# provided by the remocon client.

string name
uuid_msgs/UniqueID id

rocon_std_msgs/PlatformInfo platform_info

# Application that is being run by the interactive client.
# RemoconApp app
string app_name
