function [data, info] = setMotorModeRequest
%SetMotorMode gives an empty data for robotnik_msgs/SetMotorModeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetMotorModeRequest';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/SetMotorModeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'mode';
