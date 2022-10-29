function [data, info] = quaternion
%Quaternion gives an empty data for geometry_msgs/Quaternion

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Quaternion';
[data.x, info.x] = ros.internal.ros2.messages.ros2.default_type('double',1,0, NaN, [0]);
[data.y, info.y] = ros.internal.ros2.messages.ros2.default_type('double',1,0, NaN, [0]);
[data.z, info.z] = ros.internal.ros2.messages.ros2.default_type('double',1,0, NaN, [0]);
[data.w, info.w] = ros.internal.ros2.messages.ros2.default_type('double',1,0, NaN, [1]);
info.MessageType = 'geometry_msgs/Quaternion';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'z';
info.MatPath{4} = 'w';
