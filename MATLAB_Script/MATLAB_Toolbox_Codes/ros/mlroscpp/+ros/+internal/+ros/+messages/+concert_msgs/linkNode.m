function [data, info] = linkNode
%LinkNode gives an empty data for concert_msgs/LinkNode

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/LinkNode';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.Tuple, info.Tuple] = ros.internal.ros.messages.ros.char('string',0);
[data.UNLIMITEDRESOURCE, info.UNLIMITEDRESOURCE] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
[data.Min, info.Min] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Max, info.Max] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.ForceNameMatching, info.ForceNameMatching] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'concert_msgs/LinkNode';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'id';
info.MatPath{2} = 'tuple';
info.MatPath{3} = 'UNLIMITED_RESOURCE';
info.MatPath{4} = 'min';
info.MatPath{5} = 'max';
info.MatPath{6} = 'force_name_matching';
