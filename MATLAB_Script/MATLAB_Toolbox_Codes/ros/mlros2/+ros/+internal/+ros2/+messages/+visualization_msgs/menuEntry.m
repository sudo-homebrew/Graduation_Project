function [data, info] = menuEntry
%MenuEntry gives an empty data for visualization_msgs/MenuEntry

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visualization_msgs/MenuEntry';
[data.id, info.id] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.parent_id, info.parent_id] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.title, info.title] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.command, info.command] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.FEEDBACK, info.FEEDBACK] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.ROSRUN, info.ROSRUN] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.ROSLAUNCH, info.ROSLAUNCH] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.command_type, info.command_type] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
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
