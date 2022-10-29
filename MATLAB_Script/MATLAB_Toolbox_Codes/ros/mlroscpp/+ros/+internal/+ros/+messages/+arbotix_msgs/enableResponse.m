function [data, info] = enableResponse
%Enable gives an empty data for arbotix_msgs/EnableResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'arbotix_msgs/EnableResponse';
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'arbotix_msgs/EnableResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'state';
