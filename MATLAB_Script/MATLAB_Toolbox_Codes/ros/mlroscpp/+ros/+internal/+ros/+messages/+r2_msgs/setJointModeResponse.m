function [data, info] = setJointModeResponse
%SetJointMode gives an empty data for r2_msgs/SetJointModeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/SetJointModeResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'r2_msgs/SetJointModeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';
