function [data, info] = currentStatus
%CurrentStatus gives an empty data for scheduler_msgs/CurrentStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'scheduler_msgs/CurrentStatus';
[data.Uri, info.Uri] = ros.internal.ros.messages.ros.char('string',0);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.AVAILABLE, info.AVAILABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ALLOCATED, info.ALLOCATED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.MISSING, info.MISSING] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.GONE, info.GONE] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Owner, info.Owner] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Owner.MLdataType = 'struct';
[data.Priority, info.Priority] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Rapps, info.Rapps] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'scheduler_msgs/CurrentStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'uri';
info.MatPath{2} = 'status';
info.MatPath{3} = 'AVAILABLE';
info.MatPath{4} = 'ALLOCATED';
info.MatPath{5} = 'MISSING';
info.MatPath{6} = 'GONE';
info.MatPath{7} = 'owner';
info.MatPath{8} = 'owner.uuid';
info.MatPath{9} = 'priority';
info.MatPath{10} = 'rapps';
