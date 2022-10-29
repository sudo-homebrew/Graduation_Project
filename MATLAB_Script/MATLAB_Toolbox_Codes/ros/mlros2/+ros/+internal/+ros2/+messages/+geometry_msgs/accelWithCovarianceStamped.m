function [data, info] = accelWithCovarianceStamped
%AccelWithCovarianceStamped gives an empty data for geometry_msgs/AccelWithCovarianceStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/AccelWithCovarianceStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.accel, info.accel] = ros.internal.ros2.messages.geometry_msgs.accelWithCovariance;
info.accel.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/AccelWithCovarianceStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'accel';
info.MatPath{7} = 'accel.accel';
info.MatPath{8} = 'accel.accel.linear';
info.MatPath{9} = 'accel.accel.linear.x';
info.MatPath{10} = 'accel.accel.linear.y';
info.MatPath{11} = 'accel.accel.linear.z';
info.MatPath{12} = 'accel.accel.angular';
info.MatPath{13} = 'accel.accel.angular.x';
info.MatPath{14} = 'accel.accel.angular.y';
info.MatPath{15} = 'accel.accel.angular.z';
info.MatPath{16} = 'accel.covariance';
