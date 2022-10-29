function [data, info] = accelStamped
%AccelStamped gives an empty data for geometry_msgs/AccelStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/AccelStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Accel, info.Accel] = ros.internal.ros.messages.geometry_msgs.accel;
info.Accel.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/AccelStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'accel';
info.MatPath{8} = 'accel.linear';
info.MatPath{9} = 'accel.linear.x';
info.MatPath{10} = 'accel.linear.y';
info.MatPath{11} = 'accel.linear.z';
info.MatPath{12} = 'accel.angular';
info.MatPath{13} = 'accel.angular.x';
info.MatPath{14} = 'accel.angular.y';
info.MatPath{15} = 'accel.angular.z';
