function [data, info] = simpleInviteRequest
%SimpleInvite gives an empty data for rocon_app_manager_msgs/SimpleInviteRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/SimpleInviteRequest';
[data.Cancel, info.Cancel] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rocon_app_manager_msgs/SimpleInviteRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'cancel';
