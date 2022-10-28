function [data, info] = accelWithCovariance
%AccelWithCovariance gives an empty data for geometry_msgs/AccelWithCovariance

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/AccelWithCovariance';
[data.accel, info.accel] = ros.internal.ros2.messages.geometry_msgs.accel;
info.accel.MLdataType = 'struct';
[data.covariance, info.covariance] = ros.internal.ros2.messages.ros2.default_type('double',36,0);
info.MessageType = 'geometry_msgs/AccelWithCovariance';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'accel';
info.MatPath{2} = 'accel.linear';
info.MatPath{3} = 'accel.linear.x';
info.MatPath{4} = 'accel.linear.y';
info.MatPath{5} = 'accel.linear.z';
info.MatPath{6} = 'accel.angular';
info.MatPath{7} = 'accel.angular.x';
info.MatPath{8} = 'accel.angular.y';
info.MatPath{9} = 'accel.angular.z';
info.MatPath{10} = 'covariance';
