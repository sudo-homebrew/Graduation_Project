# Used by the remocons to inform the concert of it's current status. They should
# publish this as a latched publisher.

rocon_std_msgs/PlatformInfo platform_info
# This should be a '32 character Type 4 uuid hex string'
string uuid
# We should be using this, but java can't handle the type, reintegrate when we bugfix.
# uuid_msgs/UniqueID uuid
bool running_app

# Todo: could provide more information than the hash here for easier introspection
int32 hash

# rocon version compatibility identifier (used when connecting to concerts)
string version
