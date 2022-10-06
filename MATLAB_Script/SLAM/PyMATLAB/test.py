import matlab.engine
eng = matlab.engine.start_matlab('-nojvm')
result = eng.PySLAM(25, 'matlab_example_robot', 10)

pause(4)

occmap = eng.PyGetMap()

print(occmap)

pause(4)

print(occmap)
print(result)

eng.quit()
