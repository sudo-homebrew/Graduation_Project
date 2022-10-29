function [data, info] = twistStamped
%TwistStamped gives an empty data for geometry_msgs/TwistStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/TwistStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.twist, info.twist] = ros.internal.ros2.messages.geometry_msgs.twist;
info.twist.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/TwistStamped';
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
info.MatPath{6} = 'twist';
info.MatPath{7} = 'twist.linear';
info.MatPath{8} = 'twist.linear.x';
info.MatPath{9} = 'twist.linear.y';
info.MatPath{10} = 'twist.linear.z';
info.MatPath{11} = 'twist.angular';
info.MatPath{12} = 'twist.angular.x';
info.MatPath{13} = 'twist.angular.y';
info.MatPath{14} = 'twist.angular.z';
