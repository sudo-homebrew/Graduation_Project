function [data, info] = platformInfo
%PlatformInfo gives an empty data for rocon_app_manager_msgs/PlatformInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/PlatformInfo';
[data.PLATFORMANY, info.PLATFORMANY] = ros.internal.ros.messages.ros.char('string',0);
[data.PLATFORMANY, info.PLATFORMANY] = ros.internal.ros.messages.ros.char('string',1,'*');
[data.PLATFORMLINUX, info.PLATFORMLINUX] = ros.internal.ros.messages.ros.char('string',0);
[data.PLATFORMLINUX, info.PLATFORMLINUX] = ros.internal.ros.messages.ros.char('string',1,'linux');
[data.PLATFORMWINDOZE, info.PLATFORMWINDOZE] = ros.internal.ros.messages.ros.char('string',0);
[data.PLATFORMWINDOZE, info.PLATFORMWINDOZE] = ros.internal.ros.messages.ros.char('string',1,'windoze');
[data.PLATFORMANDROID, info.PLATFORMANDROID] = ros.internal.ros.messages.ros.char('string',0);
[data.PLATFORMANDROID, info.PLATFORMANDROID] = ros.internal.ros.messages.ros.char('string',1,'android');
[data.SYSTEMCUSTOM, info.SYSTEMCUSTOM] = ros.internal.ros.messages.ros.char('string',0);
[data.SYSTEMCUSTOM, info.SYSTEMCUSTOM] = ros.internal.ros.messages.ros.char('string',1,'custom');
[data.SYSTEMROS, info.SYSTEMROS] = ros.internal.ros.messages.ros.char('string',0);
[data.SYSTEMROS, info.SYSTEMROS] = ros.internal.ros.messages.ros.char('string',1,'ros');
[data.SYSTEMOPROS, info.SYSTEMOPROS] = ros.internal.ros.messages.ros.char('string',0);
[data.SYSTEMOPROS, info.SYSTEMOPROS] = ros.internal.ros.messages.ros.char('string',1,'opros');
[data.ROBOTANY, info.ROBOTANY] = ros.internal.ros.messages.ros.char('string',0);
[data.ROBOTANY, info.ROBOTANY] = ros.internal.ros.messages.ros.char('string',1,'*');
[data.ROBOTPC, info.ROBOTPC] = ros.internal.ros.messages.ros.char('string',0);
[data.ROBOTPC, info.ROBOTPC] = ros.internal.ros.messages.ros.char('string',1,'pc');
[data.ROBOTROBOSEM, info.ROBOTROBOSEM] = ros.internal.ros.messages.ros.char('string',0);
[data.ROBOTROBOSEM, info.ROBOTROBOSEM] = ros.internal.ros.messages.ros.char('string',1,'robosem');
[data.ROBOTKOBUKI, info.ROBOTKOBUKI] = ros.internal.ros.messages.ros.char('string',0);
[data.ROBOTKOBUKI, info.ROBOTKOBUKI] = ros.internal.ros.messages.ros.char('string',1,'kobuki');
[data.ROBOTTURTLEBOT, info.ROBOTTURTLEBOT] = ros.internal.ros.messages.ros.char('string',0);
[data.ROBOTTURTLEBOT, info.ROBOTTURTLEBOT] = ros.internal.ros.messages.ros.char('string',1,'turtlebot');
[data.Platform, info.Platform] = ros.internal.ros.messages.ros.char('string',0);
[data.System, info.System] = ros.internal.ros.messages.ros.char('string',0);
[data.Robot, info.Robot] = ros.internal.ros.messages.ros.char('string',0);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Icon, info.Icon] = ros.internal.ros.messages.rocon_app_manager_msgs.icon;
info.Icon.MLdataType = 'struct';
info.MessageType = 'rocon_app_manager_msgs/PlatformInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,19);
info.MatPath{1} = 'PLATFORM_ANY';
info.MatPath{2} = 'PLATFORM_LINUX';
info.MatPath{3} = 'PLATFORM_WINDOZE';
info.MatPath{4} = 'PLATFORM_ANDROID';
info.MatPath{5} = 'SYSTEM_CUSTOM';
info.MatPath{6} = 'SYSTEM_ROS';
info.MatPath{7} = 'SYSTEM_OPROS';
info.MatPath{8} = 'ROBOT_ANY';
info.MatPath{9} = 'ROBOT_PC';
info.MatPath{10} = 'ROBOT_ROBOSEM';
info.MatPath{11} = 'ROBOT_KOBUKI';
info.MatPath{12} = 'ROBOT_TURTLEBOT';
info.MatPath{13} = 'platform';
info.MatPath{14} = 'system';
info.MatPath{15} = 'robot';
info.MatPath{16} = 'name';
info.MatPath{17} = 'icon';
info.MatPath{18} = 'icon.format';
info.MatPath{19} = 'icon.data';