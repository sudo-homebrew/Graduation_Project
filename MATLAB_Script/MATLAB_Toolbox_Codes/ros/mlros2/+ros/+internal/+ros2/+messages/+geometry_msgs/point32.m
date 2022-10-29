function [data, info] = point32
%Point32 gives an empty data for geometry_msgs/Point32

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Point32';
[data.x, info.x] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.y, info.y] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.z, info.z] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'geometry_msgs/Point32';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'z';
