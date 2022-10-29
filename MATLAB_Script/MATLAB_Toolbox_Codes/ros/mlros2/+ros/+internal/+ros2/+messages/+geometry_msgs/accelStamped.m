function [data, info] = accelStamped
%AccelStamped gives an empty data for geometry_msgs/AccelStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/AccelStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.accel, info.accel] = ros.internal.ros2.messages.geometry_msgs.accel;
info.accel.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/AccelStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'accel';
info.MatPath{7} = 'accel.linear';
info.MatPath{8} = 'accel.linear.x';
info.MatPath{9} = 'accel.linear.y';
info.MatPath{10} = 'accel.linear.z';
info.MatPath{11} = 'accel.angular';
info.MatPath{12} = 'accel.angular.x';
info.MatPath{13} = 'accel.angular.y';
info.MatPath{14} = 'accel.angular.z';
