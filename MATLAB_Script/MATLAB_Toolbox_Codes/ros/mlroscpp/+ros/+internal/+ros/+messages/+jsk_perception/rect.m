function [data, info] = rect
%Rect gives an empty data for jsk_perception/Rect

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/Rect';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'jsk_perception/Rect';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'width';
info.MatPath{4} = 'height';
