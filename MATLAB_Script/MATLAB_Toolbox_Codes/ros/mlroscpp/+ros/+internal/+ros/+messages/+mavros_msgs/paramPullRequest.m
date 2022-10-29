function [data, info] = paramPullRequest
%ParamPull gives an empty data for mavros_msgs/ParamPullRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ParamPullRequest';
[data.ForcePull, info.ForcePull] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'mavros_msgs/ParamPullRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'force_pull';
