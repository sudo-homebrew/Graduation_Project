import matlab.engine
import time

print("Launching MATLAB")
eng = matlab.engine.start_matlab()
print("MATLAB has launched")

# How to use PySLAM
# PySLAM(ROS_DOMAIN_ID, ROS2DomainID, Duration)
result = eng.PySLAM(25, 'matlab_example_robot', 10)
print("Starting PySLAM")

time.sleep(4)

occmap = eng.PyGetMap()

print(occmap)

time.sleep(4)

print(occmap)
print(result)

eng.quit()
