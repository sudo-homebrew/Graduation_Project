function [data, info] = setMotorPIDRequest
%SetMotorPID gives an empty data for robotnik_msgs/SetMotorPIDRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetMotorPIDRequest';
[data.Pid, info.Pid] = ros.internal.ros.messages.robotnik_msgs.motorPID;
info.Pid.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/SetMotorPIDRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'pid';
info.MatPath{2} = 'pid.can_id';
info.MatPath{3} = 'pid.name';
info.MatPath{4} = 'pid.kp';
info.MatPath{5} = 'pid.ki';
info.MatPath{6} = 'pid.kd';
