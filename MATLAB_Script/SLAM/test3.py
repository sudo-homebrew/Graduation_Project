import matlab.engine
import time

print("Launching MATLAB")
eng = matlab.engine.start_matlab()
print("MATLAB has launched")



print("Starting PySLAM")
# How to use PySLAM
# PySLAM(ROS_DOMAIN_ID, ROS2DomainID, Duration)
result = eng.PySLAM(25, 'matlab_example_robot', 91)
print("PySLAM has started")

time.sleep(90)

if result:
    print("Process has succesfully  done")
else:
    print(2, "E:Process has proble")

eng.quit()
