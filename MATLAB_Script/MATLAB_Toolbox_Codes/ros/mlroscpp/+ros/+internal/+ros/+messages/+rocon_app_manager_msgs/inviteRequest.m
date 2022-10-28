function [data, info] = inviteRequest
%Invite gives an empty data for rocon_app_manager_msgs/InviteRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/InviteRequest';
[data.RemoteTargetName, info.RemoteTargetName] = ros.internal.ros.messages.ros.char('string',0);
[data.ApplicationNamespace, info.ApplicationNamespace] = ros.internal.ros.messages.ros.char('string',0);
[data.Cancel, info.Cancel] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rocon_app_manager_msgs/InviteRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'remote_target_name';
info.MatPath{2} = 'application_namespace';
info.MatPath{3} = 'cancel';
