function [data, info] = motorPID
%MotorPID gives an empty data for robotnik_msgs/MotorPID

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/MotorPID';
[data.CanId, info.CanId] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Kp, info.Kp] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Ki, info.Ki] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Kd, info.Kd] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'robotnik_msgs/MotorPID';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'can_id';
info.MatPath{2} = 'name';
info.MatPath{3} = 'kp';
info.MatPath{4} = 'ki';
info.MatPath{5} = 'kd';
