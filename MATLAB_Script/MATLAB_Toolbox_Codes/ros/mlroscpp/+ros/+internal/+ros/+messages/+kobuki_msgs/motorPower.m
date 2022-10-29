function [data, info] = motorPower
%MotorPower gives an empty data for kobuki_msgs/MotorPower

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/MotorPower';
[data.OFF, info.OFF] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ON, info.ON] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/MotorPower';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'OFF';
info.MatPath{2} = 'ON';
info.MatPath{3} = 'state';
