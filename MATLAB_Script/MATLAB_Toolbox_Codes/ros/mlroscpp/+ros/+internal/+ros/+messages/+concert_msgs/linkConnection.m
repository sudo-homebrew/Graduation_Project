function [data, info] = linkConnection
%LinkConnection gives an empty data for concert_msgs/LinkConnection

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/LinkConnection';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'concert_msgs/LinkConnection';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'id';
info.MatPath{2} = 'type';
