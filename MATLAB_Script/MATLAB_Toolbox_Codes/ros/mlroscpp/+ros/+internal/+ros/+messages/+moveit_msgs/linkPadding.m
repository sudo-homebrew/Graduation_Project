function [data, info] = linkPadding
%LinkPadding gives an empty data for moveit_msgs/LinkPadding

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/LinkPadding';
[data.LinkName, info.LinkName] = ros.internal.ros.messages.ros.char('string',0);
[data.Padding, info.Padding] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'moveit_msgs/LinkPadding';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'link_name';
info.MatPath{2} = 'padding';
