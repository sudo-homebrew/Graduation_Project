function [data, info] = objectInfo
%ObjectInfo gives an empty data for hector_worldmodel_msgs/ObjectInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/ObjectInfo';
[data.ClassId, info.ClassId] = ros.internal.ros.messages.ros.char('string',0);
[data.ObjectId, info.ObjectId] = ros.internal.ros.messages.ros.char('string',0);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Support, info.Support] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'hector_worldmodel_msgs/ObjectInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'class_id';
info.MatPath{2} = 'object_id';
info.MatPath{3} = 'name';
info.MatPath{4} = 'support';
