function [data, info] = inertiaStamped
%InertiaStamped gives an empty data for geometry_msgs/InertiaStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/InertiaStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Inertia, info.Inertia] = ros.internal.ros.messages.geometry_msgs.inertia;
info.Inertia.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/InertiaStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'inertia';
info.MatPath{8} = 'inertia.m';
info.MatPath{9} = 'inertia.com';
info.MatPath{10} = 'inertia.com.x';
info.MatPath{11} = 'inertia.com.y';
info.MatPath{12} = 'inertia.com.z';
info.MatPath{13} = 'inertia.ixx';
info.MatPath{14} = 'inertia.ixy';
info.MatPath{15} = 'inertia.ixz';
info.MatPath{16} = 'inertia.iyy';
info.MatPath{17} = 'inertia.iyz';
info.MatPath{18} = 'inertia.izz';
