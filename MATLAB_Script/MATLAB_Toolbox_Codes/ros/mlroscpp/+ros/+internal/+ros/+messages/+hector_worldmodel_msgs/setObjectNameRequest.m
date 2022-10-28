function [data, info] = setObjectNameRequest
%SetObjectName gives an empty data for hector_worldmodel_msgs/SetObjectNameRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/SetObjectNameRequest';
[data.ObjectId, info.ObjectId] = ros.internal.ros.messages.ros.char('string',0);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'hector_worldmodel_msgs/SetObjectNameRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'object_id';
info.MatPath{2} = 'name';
