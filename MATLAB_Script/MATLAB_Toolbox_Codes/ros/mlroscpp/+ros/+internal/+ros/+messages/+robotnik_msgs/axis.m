function [data, info] = axis
%Axis gives an empty data for robotnik_msgs/Axis

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Axis';
[data.Pan, info.Pan] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Tilt, info.Tilt] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Zoom, info.Zoom] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Focus, info.Focus] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Brightness, info.Brightness] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Iris, info.Iris] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Autofocus, info.Autofocus] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Autoiris, info.Autoiris] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/Axis';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'pan';
info.MatPath{2} = 'tilt';
info.MatPath{3} = 'zoom';
info.MatPath{4} = 'focus';
info.MatPath{5} = 'brightness';
info.MatPath{6} = 'iris';
info.MatPath{7} = 'autofocus';
info.MatPath{8} = 'autoiris';
