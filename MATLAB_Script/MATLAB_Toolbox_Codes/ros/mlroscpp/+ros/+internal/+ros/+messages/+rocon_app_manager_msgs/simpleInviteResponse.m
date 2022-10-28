function [data, info] = simpleInviteResponse
%SimpleInvite gives an empty data for rocon_app_manager_msgs/SimpleInviteResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/SimpleInviteResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rocon_app_manager_msgs/SimpleInviteResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';
