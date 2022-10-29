function [data, info] = simpleMotorFlasherRequest
%SimpleMotorFlasher gives an empty data for sr_robot_msgs/SimpleMotorFlasherRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/SimpleMotorFlasherRequest';
[data.Firmware, info.Firmware] = ros.internal.ros.messages.ros.char('string',0);
[data.MotorId, info.MotorId] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'sr_robot_msgs/SimpleMotorFlasherRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'firmware';
info.MatPath{2} = 'motor_id';
