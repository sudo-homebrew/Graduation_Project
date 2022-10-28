# Represents a command for the four corner body lights, indicators on the rear
# and beeper/beacon combination on the arch.
uint8 LIGHTS_FRONT_LEFT=0   # Only white
uint8 LIGHTS_FRONT_RIGHT=1  # Only white
uint8 LIGHTS_REAR_LEFT=2    # Only red
uint8 LIGHTS_REAR_RIGHT=3   # Only red

uint8[4] body_lights    # Body lights on the Grizzly
uint8 beacon            # Overhead beacon light
uint8 beep              # Noisemaker sound
