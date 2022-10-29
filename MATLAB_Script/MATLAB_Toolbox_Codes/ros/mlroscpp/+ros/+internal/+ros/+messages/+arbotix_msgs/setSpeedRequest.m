function [data, info] = setSpeedRequest
%SetSpeed gives an empty data for arbotix_msgs/SetSpeedRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'arbotix_msgs/SetSpeedRequest';
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'arbotix_msgs/SetSpeedRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'speed';
