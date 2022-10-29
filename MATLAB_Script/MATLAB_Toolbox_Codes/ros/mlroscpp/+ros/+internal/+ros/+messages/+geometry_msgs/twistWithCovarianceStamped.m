function [data, info] = twistWithCovarianceStamped
%TwistWithCovarianceStamped gives an empty data for geometry_msgs/TwistWithCovarianceStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/TwistWithCovarianceStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Twist, info.Twist] = ros.internal.ros.messages.geometry_msgs.twistWithCovariance;
info.Twist.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/TwistWithCovarianceStamped';
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
info.MatPath{7} = 'twist';
info.MatPath{8} = 'twist.twist';
info.MatPath{9} = 'twist.twist.linear';
info.MatPath{10} = 'twist.twist.linear.x';
info.MatPath{11} = 'twist.twist.linear.y';
info.MatPath{12} = 'twist.twist.linear.z';
info.MatPath{13} = 'twist.twist.angular';
info.MatPath{14} = 'twist.twist.angular.x';
info.MatPath{15} = 'twist.twist.angular.y';
info.MatPath{16} = 'twist.twist.angular.z';
info.MatPath{17} = 'twist.covariance';
