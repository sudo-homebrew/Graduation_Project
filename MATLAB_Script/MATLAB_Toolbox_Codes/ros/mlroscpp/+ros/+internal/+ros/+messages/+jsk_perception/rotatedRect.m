function [data, info] = rotatedRect
%RotatedRect gives an empty data for jsk_perception/RotatedRect

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/RotatedRect';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Angle, info.Angle] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_perception/RotatedRect';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'width';
info.MatPath{4} = 'height';
info.MatPath{5} = 'angle';
