function [data, info] = pR2GripperForceServoCommand
%PR2GripperForceServoCommand gives an empty data for pr2_gripper_sensor_msgs/PR2GripperForceServoCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperForceServoCommand';
[data.FingertipForce, info.FingertipForce] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperForceServoCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'fingertip_force';
