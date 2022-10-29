function [data, info] = plane
%Plane gives an empty data for shape_msgs/Plane

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shape_msgs/Plane';
[data.coef, info.coef] = ros.internal.ros2.messages.ros2.default_type('double',4,0);
info.MessageType = 'shape_msgs/Plane';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'coef';
