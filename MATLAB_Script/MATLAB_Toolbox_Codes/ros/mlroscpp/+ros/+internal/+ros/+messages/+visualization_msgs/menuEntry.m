function [data, info] = menuEntry
%MenuEntry gives an empty data for visualization_msgs/MenuEntry

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visualization_msgs/MenuEntry';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.ParentId, info.ParentId] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Title, info.Title] = ros.internal.ros.messages.ros.char('string',0);
[data.Command, info.Command] = ros.internal.ros.messages.ros.char('string',0);
[data.FEEDBACK, info.FEEDBACK] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ROSRUN, info.ROSRUN] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.ROSLAUNCH, info.ROSLAUNCH] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.CommandType, info.CommandType] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'visualization_msgs/MenuEntry';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'id';
info.MatPath{2} = 'parent_id';
info.MatPath{3} = 'title';
info.MatPath{4} = 'command';
info.MatPath{5} = 'FEEDBACK';
info.MatPath{6} = 'ROSRUN';
info.MatPath{7} = 'ROSLAUNCH';
info.MatPath{8} = 'command_type';
