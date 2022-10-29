function [data, info] = setMotorStatusRequest
%SetMotorStatus gives an empty data for robotnik_msgs/SetMotorStatusRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetMotorStatusRequest';
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'robotnik_msgs/SetMotorStatusRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'status';
