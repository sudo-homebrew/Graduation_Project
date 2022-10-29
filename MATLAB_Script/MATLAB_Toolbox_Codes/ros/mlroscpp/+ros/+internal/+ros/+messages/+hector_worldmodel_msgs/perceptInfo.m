function [data, info] = perceptInfo
%PerceptInfo gives an empty data for hector_worldmodel_msgs/PerceptInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/PerceptInfo';
[data.ClassId, info.ClassId] = ros.internal.ros.messages.ros.char('string',0);
[data.ClassSupport, info.ClassSupport] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ObjectId, info.ObjectId] = ros.internal.ros.messages.ros.char('string',0);
[data.ObjectSupport, info.ObjectSupport] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'hector_worldmodel_msgs/PerceptInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'class_id';
info.MatPath{2} = 'class_support';
info.MatPath{3} = 'object_id';
info.MatPath{4} = 'object_support';
info.MatPath{5} = 'name';
