function [data, info] = waypoint
%Waypoint gives an empty data for mavros_msgs/Waypoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/Waypoint';
[data.Frame, info.Frame] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.FRAMEGLOBAL, info.FRAMEGLOBAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.FRAMELOCALNED, info.FRAMELOCALNED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.FRAMEMISSION, info.FRAMEMISSION] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.FRAMEGLOBALRELALT, info.FRAMEGLOBALRELALT] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.FRAMELOCALENU, info.FRAMELOCALENU] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.IsCurrent, info.IsCurrent] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Autocontinue, info.Autocontinue] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Param1, info.Param1] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param2, info.Param2] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param3, info.Param3] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param4, info.Param4] = ros.internal.ros.messages.ros.default_type('single',1);
[data.XLat, info.XLat] = ros.internal.ros.messages.ros.default_type('double',1);
[data.YLong, info.YLong] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ZAlt, info.ZAlt] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'mavros_msgs/Waypoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'frame';
info.MatPath{2} = 'FRAME_GLOBAL';
info.MatPath{3} = 'FRAME_LOCAL_NED';
info.MatPath{4} = 'FRAME_MISSION';
info.MatPath{5} = 'FRAME_GLOBAL_REL_ALT';
info.MatPath{6} = 'FRAME_LOCAL_ENU';
info.MatPath{7} = 'command';
info.MatPath{8} = 'is_current';
info.MatPath{9} = 'autocontinue';
info.MatPath{10} = 'param1';
info.MatPath{11} = 'param2';
info.MatPath{12} = 'param3';
info.MatPath{13} = 'param4';
info.MatPath{14} = 'x_lat';
info.MatPath{15} = 'y_long';
info.MatPath{16} = 'z_alt';