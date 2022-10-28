function [data, info] = inviteResponse
%Invite gives an empty data for concert_msgs/InviteResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/InviteResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'concert_msgs/InviteResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';
