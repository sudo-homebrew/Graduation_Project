function [data, info] = inertiaStamped
%InertiaStamped gives an empty data for geometry_msgs/InertiaStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/InertiaStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.inertia, info.inertia] = ros.internal.ros2.messages.geometry_msgs.inertia;
info.inertia.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/InertiaStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'inertia';
info.MatPath{7} = 'inertia.m';
info.MatPath{8} = 'inertia.com';
info.MatPath{9} = 'inertia.com.x';
info.MatPath{10} = 'inertia.com.y';
info.MatPath{11} = 'inertia.com.z';
info.MatPath{12} = 'inertia.ixx';
info.MatPath{13} = 'inertia.ixy';
info.MatPath{14} = 'inertia.ixz';
info.MatPath{15} = 'inertia.iyy';
info.MatPath{16} = 'inertia.iyz';
info.MatPath{17} = 'inertia.izz';
