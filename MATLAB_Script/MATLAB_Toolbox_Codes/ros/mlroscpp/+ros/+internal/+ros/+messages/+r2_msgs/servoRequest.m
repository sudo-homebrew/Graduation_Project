function [data, info] = servoRequest
%Servo gives an empty data for r2_msgs/ServoRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/ServoRequest';
[data.Joint, info.Joint] = ros.internal.ros.messages.ros.char('string',0);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'r2_msgs/ServoRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'joint';
info.MatPath{2} = 'state';
