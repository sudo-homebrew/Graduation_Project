function [data, info] = pR2GripperPressureData
%PR2GripperPressureData gives an empty data for pr2_gripper_sensor_msgs/PR2GripperPressureData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperPressureData';
[data.PressureLeft, info.PressureLeft] = ros.internal.ros.messages.ros.default_type('double',22);
[data.PressureRight, info.PressureRight] = ros.internal.ros.messages.ros.default_type('double',22);
[data.Rostime, info.Rostime] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperPressureData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'pressure_left';
info.MatPath{2} = 'pressure_right';
info.MatPath{3} = 'rostime';
