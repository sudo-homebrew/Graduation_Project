function [data, info] = quaternionStamped
%QuaternionStamped gives an empty data for geometry_msgs/QuaternionStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/QuaternionStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.quaternion, info.quaternion] = ros.internal.ros2.messages.geometry_msgs.quaternion;
info.quaternion.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/QuaternionStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'quaternion';
info.MatPath{7} = 'quaternion.x';
info.MatPath{8} = 'quaternion.y';
info.MatPath{9} = 'quaternion.z';
info.MatPath{10} = 'quaternion.w';
