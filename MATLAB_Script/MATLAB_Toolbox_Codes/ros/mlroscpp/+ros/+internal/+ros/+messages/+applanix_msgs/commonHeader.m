function [data, info] = commonHeader
%CommonHeader gives an empty data for applanix_msgs/CommonHeader

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/CommonHeader';
[data.STARTGROUP, info.STARTGROUP] = ros.internal.ros.messages.ros.char('string',0);
[data.STARTGROUP, info.STARTGROUP] = ros.internal.ros.messages.ros.char('string',1,'$GRP');
[data.STARTMESSAGE, info.STARTMESSAGE] = ros.internal.ros.messages.ros.char('string',0);
[data.STARTMESSAGE, info.STARTMESSAGE] = ros.internal.ros.messages.ros.char('string',1,'$MSG');
[data.Start, info.Start] = ros.internal.ros.messages.ros.default_type('uint8',4);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Length, info.Length] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'applanix_msgs/CommonHeader';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'START_GROUP';
info.MatPath{2} = 'START_MESSAGE';
info.MatPath{3} = 'start';
info.MatPath{4} = 'id';
info.MatPath{5} = 'length';
