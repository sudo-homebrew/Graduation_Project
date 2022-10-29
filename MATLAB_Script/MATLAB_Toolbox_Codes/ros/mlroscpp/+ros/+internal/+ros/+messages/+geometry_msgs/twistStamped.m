function [data, info] = twistStamped
%TwistStamped gives an empty data for geometry_msgs/TwistStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/TwistStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Twist, info.Twist] = ros.internal.ros.messages.geometry_msgs.twist;
info.Twist.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/TwistStamped';
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
info.MatPath{7} = 'twist';
info.MatPath{8} = 'twist.linear';
info.MatPath{9} = 'twist.linear.x';
info.MatPath{10} = 'twist.linear.y';
info.MatPath{11} = 'twist.linear.z';
info.MatPath{12} = 'twist.angular';
info.MatPath{13} = 'twist.angular.x';
info.MatPath{14} = 'twist.angular.y';
info.MatPath{15} = 'twist.angular.z';
