function [data, info] = pR2GripperFindContactCommand
%PR2GripperFindContactCommand gives an empty data for pr2_gripper_sensor_msgs/PR2GripperFindContactCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperFindContactCommand';
[data.ZeroFingertipSensors, info.ZeroFingertipSensors] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ContactConditions, info.ContactConditions] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.BOTH, info.BOTH] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.LEFT, info.LEFT] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.RIGHT, info.RIGHT] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.EITHER, info.EITHER] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperFindContactCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'zero_fingertip_sensors';
info.MatPath{2} = 'contact_conditions';
info.MatPath{3} = 'BOTH';
info.MatPath{4} = 'LEFT';
info.MatPath{5} = 'RIGHT';
info.MatPath{6} = 'EITHER';
