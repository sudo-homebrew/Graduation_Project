function [data, info] = imagePoint
%ImagePoint gives an empty data for image_cb_detector/ImagePoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_cb_detector/ImagePoint';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'image_cb_detector/ImagePoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';