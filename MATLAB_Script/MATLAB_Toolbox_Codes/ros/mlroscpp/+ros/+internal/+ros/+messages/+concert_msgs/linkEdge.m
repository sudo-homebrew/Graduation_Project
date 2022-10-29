function [data, info] = linkEdge
%LinkEdge gives an empty data for concert_msgs/LinkEdge

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/LinkEdge';
[data.Start, info.Start] = ros.internal.ros.messages.ros.char('string',0);
[data.Finish, info.Finish] = ros.internal.ros.messages.ros.char('string',0);
[data.RemapFrom, info.RemapFrom] = ros.internal.ros.messages.ros.char('string',0);
[data.RemapTo, info.RemapTo] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'concert_msgs/LinkEdge';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'start';
info.MatPath{2} = 'finish';
info.MatPath{3} = 'remap_from';
info.MatPath{4} = 'remap_to';
