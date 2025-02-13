function [data, info] = overlayMenu
%OverlayMenu gives an empty data for jsk_rviz_plugins/OverlayMenu

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_rviz_plugins/OverlayMenu';
[data.ACTIONSELECT, info.ACTIONSELECT] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.ACTIONCLOSE, info.ACTIONCLOSE] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.Action, info.Action] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.CurrentIndex, info.CurrentIndex] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Menus, info.Menus] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Title, info.Title] = ros.internal.ros.messages.ros.char('string',0);
[data.BgColor, info.BgColor] = ros.internal.ros.messages.std_msgs.colorRGBA;
info.BgColor.MLdataType = 'struct';
[data.FgColor, info.FgColor] = ros.internal.ros.messages.std_msgs.colorRGBA;
info.FgColor.MLdataType = 'struct';
info.MessageType = 'jsk_rviz_plugins/OverlayMenu';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'ACTION_SELECT';
info.MatPath{2} = 'ACTION_CLOSE';
info.MatPath{3} = 'action';
info.MatPath{4} = 'current_index';
info.MatPath{5} = 'menus';
info.MatPath{6} = 'title';
info.MatPath{7} = 'bg_color';
info.MatPath{8} = 'bg_color.r';
info.MatPath{9} = 'bg_color.g';
info.MatPath{10} = 'bg_color.b';
info.MatPath{11} = 'bg_color.a';
info.MatPath{12} = 'fg_color';
info.MatPath{13} = 'fg_color.r';
info.MatPath{14} = 'fg_color.g';
info.MatPath{15} = 'fg_color.b';
info.MatPath{16} = 'fg_color.a';
