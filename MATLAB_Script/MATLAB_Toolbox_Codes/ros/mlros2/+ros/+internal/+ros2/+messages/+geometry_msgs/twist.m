function [data, info] = twist
%Twist gives an empty data for geometry_msgs/Twist

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Twist';
[data.linear, info.linear] = ros.internal.ros2.messages.geometry_msgs.vector3;
info.linear.MLdataType = 'struct';
[data.angular, info.angular] = ros.internal.ros2.messages.geometry_msgs.vector3;
info.angular.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/Twist';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'linear';
info.MatPath{2} = 'linear.x';
info.MatPath{3} = 'linear.y';
info.MatPath{4} = 'linear.z';
info.MatPath{5} = 'angular';
info.MatPath{6} = 'angular.x';
info.MatPath{7} = 'angular.y';
info.MatPath{8} = 'angular.z';
