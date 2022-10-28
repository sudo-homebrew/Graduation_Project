function [data, info] = objectState
%ObjectState gives an empty data for hector_worldmodel_msgs/ObjectState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/ObjectState';
[data.UNKNOWN, info.UNKNOWN] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.PENDING, info.PENDING] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.ACTIVE, info.ACTIVE] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.INACTIVE, info.INACTIVE] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.CONFIRMED, info.CONFIRMED] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
[data.DISCARDED, info.DISCARDED] = ros.internal.ros.messages.ros.default_type('int8',1, -2);
[data.APPROACHING, info.APPROACHING] = ros.internal.ros.messages.ros.default_type('int8',1, -3);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'hector_worldmodel_msgs/ObjectState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'UNKNOWN';
info.MatPath{2} = 'PENDING';
info.MatPath{3} = 'ACTIVE';
info.MatPath{4} = 'INACTIVE';
info.MatPath{5} = 'CONFIRMED';
info.MatPath{6} = 'DISCARDED';
info.MatPath{7} = 'APPROACHING';
info.MatPath{8} = 'state';
