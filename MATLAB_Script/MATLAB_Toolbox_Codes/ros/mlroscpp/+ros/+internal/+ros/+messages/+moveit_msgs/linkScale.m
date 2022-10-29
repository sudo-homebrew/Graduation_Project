function [data, info] = linkScale
%LinkScale gives an empty data for moveit_msgs/LinkScale

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/LinkScale';
[data.LinkName, info.LinkName] = ros.internal.ros.messages.ros.char('string',0);
[data.Scale, info.Scale] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'moveit_msgs/LinkScale';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'link_name';
info.MatPath{2} = 'scale';
