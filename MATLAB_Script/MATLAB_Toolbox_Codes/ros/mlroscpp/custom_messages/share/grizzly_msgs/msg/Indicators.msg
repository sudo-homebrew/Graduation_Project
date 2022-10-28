# Represents a command for the indicators on the rear panel.

uint8 INDICATOR_ON      = 255
uint8 INDICATOR_FLASH   = 15
uint8 INDICATOR_OFF     = 0

uint8 position_light    # Signal the status of the GPS system
uint8 autopilot_light   # Signal if the system is under automatic or manual control
uint8 battery_light     # Signal the level of the battery, 0 is empty and 255 is full.

