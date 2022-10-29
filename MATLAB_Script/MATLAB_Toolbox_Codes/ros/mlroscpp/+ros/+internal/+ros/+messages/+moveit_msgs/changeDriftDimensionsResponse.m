function [data, info] = changeDriftDimensionsResponse
%ChangeDriftDimensions gives an empty data for moveit_msgs/ChangeDriftDimensionsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ChangeDriftDimensionsResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'moveit_msgs/ChangeDriftDimensionsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
