function [data, info] = config
%config gives an empty data for sr_robot_msgs/config

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/config';
[data.NodeName, info.NodeName] = ros.internal.ros.messages.ros.char('string',0);
[data.ListOfParameters, info.ListOfParameters] = ros.internal.ros.messages.ros.char('string',NaN);
[data.LengthOfList, info.LengthOfList] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'sr_robot_msgs/config';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'node_name';
info.MatPath{2} = 'list_of_parameters';
info.MatPath{3} = 'length_of_list';