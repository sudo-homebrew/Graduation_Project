function [data, info] = regionOfInterest
%RegionOfInterest gives an empty data for sensor_msgs/RegionOfInterest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/RegionOfInterest';
[data.XOffset, info.XOffset] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.YOffset, info.YOffset] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.DoRectify, info.DoRectify] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'sensor_msgs/RegionOfInterest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'x_offset';
info.MatPath{2} = 'y_offset';
info.MatPath{3} = 'height';
info.MatPath{4} = 'width';
info.MatPath{5} = 'do_rectify';
