function [data, info] = touch
%Touch gives an empty data for jsk_gui_msgs/Touch

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/Touch';
[data.FingerId, info.FingerId] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ImageWidth, info.ImageWidth] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.ImageHeight, info.ImageHeight] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'jsk_gui_msgs/Touch';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'finger_id';
info.MatPath{2} = 'x';
info.MatPath{3} = 'y';
info.MatPath{4} = 'image_width';
info.MatPath{5} = 'image_height';
