function [data, info] = enableRequest
%Enable gives an empty data for arbotix_msgs/EnableRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'arbotix_msgs/EnableRequest';
[data.Enable, info.Enable] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'arbotix_msgs/EnableRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'enable';
