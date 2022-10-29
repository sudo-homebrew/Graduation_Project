function [data, info] = twistWithCovarianceStamped
%TwistWithCovarianceStamped gives an empty data for geometry_msgs/TwistWithCovarianceStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/TwistWithCovarianceStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.twist, info.twist] = ros.internal.ros2.messages.geometry_msgs.twistWithCovariance;
info.twist.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/TwistWithCovarianceStamped';
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
info.MatPath{6} = 'twist';
info.MatPath{7} = 'twist.twist';
info.MatPath{8} = 'twist.twist.linear';
info.MatPath{9} = 'twist.twist.linear.x';
info.MatPath{10} = 'twist.twist.linear.y';
info.MatPath{11} = 'twist.twist.linear.z';
info.MatPath{12} = 'twist.twist.angular';
info.MatPath{13} = 'twist.twist.angular.x';
info.MatPath{14} = 'twist.twist.angular.y';
info.MatPath{15} = 'twist.twist.angular.z';
info.MatPath{16} = 'twist.covariance';
