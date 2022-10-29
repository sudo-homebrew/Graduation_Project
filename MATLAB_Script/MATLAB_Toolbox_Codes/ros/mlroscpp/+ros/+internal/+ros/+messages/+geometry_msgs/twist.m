function [data, info] = twist
%Twist gives an empty data for geometry_msgs/Twist

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Twist';
[data.Linear, info.Linear] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Linear.MLdataType = 'struct';
[data.Angular, info.Angular] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Angular.MLdataType = 'struct';
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
