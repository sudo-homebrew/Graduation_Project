function [data, info] = uniqueID
%UniqueID gives an empty data for uuid_msgs/UniqueID

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'uuid_msgs/UniqueID';
[data.Uuid, info.Uuid] = ros.internal.ros.messages.ros.default_type('uint8',16);
info.MessageType = 'uuid_msgs/UniqueID';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'uuid';
