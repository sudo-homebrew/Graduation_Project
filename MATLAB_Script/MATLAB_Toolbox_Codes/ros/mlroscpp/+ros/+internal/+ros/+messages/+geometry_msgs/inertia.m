function [data, info] = inertia
%Inertia gives an empty data for geometry_msgs/Inertia

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Inertia';
[data.M, info.M] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Com, info.Com] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Com.MLdataType = 'struct';
[data.Ixx, info.Ixx] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Ixy, info.Ixy] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Ixz, info.Ixz] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Iyy, info.Iyy] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Iyz, info.Iyz] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Izz, info.Izz] = ros.internal.ros.messages.ros.default_type('double',1);
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
