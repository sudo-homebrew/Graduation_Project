function [data, info] = accelWithCovarianceStamped
%AccelWithCovarianceStamped gives an empty data for geometry_msgs/AccelWithCovarianceStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/AccelWithCovarianceStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Accel, info.Accel] = ros.internal.ros.messages.geometry_msgs.accelWithCovariance;
info.Accel.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/AccelWithCovarianceStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'accel';
info.MatPath{8} = 'accel.accel';
info.MatPath{9} = 'accel.accel.linear';
info.MatPath{10} = 'accel.accel.linear.x';
info.MatPath{11} = 'accel.accel.linear.y';
info.MatPath{12} = 'accel.accel.linear.z';
info.MatPath{13} = 'accel.accel.angular';
info.MatPath{14} = 'accel.accel.angular.x';
info.MatPath{15} = 'accel.accel.angular.y';
info.MatPath{16} = 'accel.accel.angular.z';
info.MatPath{17} = 'accel.covariance';
