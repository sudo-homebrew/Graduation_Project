# 
# Describes a concert client - used only on the concert side
#
# Unique names - human consumable rocon name as well as global gateway name
string name
string gateway_name

# The platform id triple (refer to concert_clients/PlatformInfo.msg for string values)
string platform
string system
string robot

# Current status of the concert client (refer to concert_msgs/ConcertClient.msg for mode strings)
string client_status
string app_status

# Depracate this
string status

# Statistics
time last_connection_timestamp

rocon_app_manager_msgs/App[] apps
