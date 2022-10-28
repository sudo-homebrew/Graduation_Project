function [data, info] = objectProperty
%ObjectProperty gives an empty data for grasping_msgs/ObjectProperty

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grasping_msgs/ObjectProperty';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'grasping_msgs/ObjectProperty';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'value';
