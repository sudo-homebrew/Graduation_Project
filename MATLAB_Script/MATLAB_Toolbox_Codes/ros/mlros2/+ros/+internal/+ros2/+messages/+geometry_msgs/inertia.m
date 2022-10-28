function [data, info] = inertia
%Inertia gives an empty data for geometry_msgs/Inertia

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Inertia';
[data.m, info.m] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.com, info.com] = ros.internal.ros2.messages.geometry_msgs.vector3;
info.com.MLdataType = 'struct';
[data.ixx, info.ixx] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.ixy, info.ixy] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.ixz, info.ixz] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.iyy, info.iyy] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.iyz, info.iyz] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.izz, info.izz] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'geometry_msgs/Inertia';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'm';
info.MatPath{2} = 'com';
info.MatPath{3} = 'com.x';
info.MatPath{4} = 'com.y';
info.MatPath{5} = 'com.z';
info.MatPath{6} = 'ixx';
info.MatPath{7} = 'ixy';
info.MatPath{8} = 'ixz';
info.MatPath{9} = 'iyy';
info.MatPath{10} = 'iyz';
info.MatPath{11} = 'izz';
