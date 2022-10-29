function [data, info] = twistWithCovariance
%TwistWithCovariance gives an empty data for geometry_msgs/TwistWithCovariance

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/TwistWithCovariance';
[data.Twist, info.Twist] = ros.internal.ros.messages.geometry_msgs.twist;
info.Twist.MLdataType = 'struct';
[data.Covariance, info.Covariance] = ros.internal.ros.messages.ros.default_type('double',36);
info.MessageType = 'geometry_msgs/TwistWithCovariance';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'twist';
info.MatPath{2} = 'twist.linear';
info.MatPath{3} = 'twist.linear.x';
info.MatPath{4} = 'twist.linear.y';
info.MatPath{5} = 'twist.linear.z';
info.MatPath{6} = 'twist.angular';
info.MatPath{7} = 'twist.angular.x';
info.MatPath{8} = 'twist.angular.y';
info.MatPath{9} = 'twist.angular.z';
info.MatPath{10} = 'covariance';
