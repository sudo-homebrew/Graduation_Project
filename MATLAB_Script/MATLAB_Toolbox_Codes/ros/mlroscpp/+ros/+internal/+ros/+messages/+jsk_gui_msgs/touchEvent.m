function [data, info] = touchEvent
%TouchEvent gives an empty data for jsk_gui_msgs/TouchEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/TouchEvent';
[data.DOWN, info.DOWN] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.UP, info.UP] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.MOVE, info.MOVE] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
[data.W, info.W] = ros.internal.ros.messages.ros.default_type('single',1);
[data.H, info.H] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'jsk_gui_msgs/TouchEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'DOWN';
info.MatPath{2} = 'UP';
info.MatPath{3} = 'MOVE';
info.MatPath{4} = 'state';
info.MatPath{5} = 'x';
info.MatPath{6} = 'y';
info.MatPath{7} = 'w';
info.MatPath{8} = 'h';
