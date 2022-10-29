# valid command IDs for the Segway
# [RMP_MOTION_CMD_ID, norm_vel_cmd, norm_yaw_cmd]
# [RMP_OMNI_MOTION_CMD_ID, norm_vel_cmd, norm_yaw_cmd, angle_cmd_deg]
# [RMP_CFG_CMD_ID, gp_cmd, gp_param]
uint32 RMP_MOTION_CMD_ID = 1280
uint32 RMP_OMNI_MOTION_CMD_ID = 1536
uint32 RMP_CFG_CMD_ID = 1281

# arg1 some possible values (can also be linear velocity )
uint8 RMP_CMD_SET_OPERATIONAL_MODE = 32
uint8 RMP_CMD_NONE = 0 

# arg2 some possible values (can also be angular velocity)
uint8 DISABLE_REQUEST   = 1
uint8 POWERDOWN_REQUEST = 2
uint8 DTZ_REQUEST       = 3
uint8 STANDBY_REQUEST   = 4
uint8 TRACTOR_REQUEST   = 5
uint8 BALANCE_REQUEST   = 6

# Message content
Header header
uint32 cmd_id
float32 arg1
float32 arg2
float32 arg3
