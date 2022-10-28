function [data, info] = inviteRequest
%Invite gives an empty data for concert_msgs/InviteRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/InviteRequest';
[data.Mastername, info.Mastername] = ros.internal.ros.messages.ros.char('string',0);
[data.Clientnames, info.Clientnames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.OkFlag, info.OkFlag] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'concert_msgs/InviteRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'mastername';
info.MatPath{2} = 'clientnames';
info.MatPath{3} = 'ok_flag';
