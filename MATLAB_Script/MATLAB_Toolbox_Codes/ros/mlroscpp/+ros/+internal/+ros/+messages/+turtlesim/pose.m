function [data, info] = pose
%Pose gives an empty data for turtlesim/Pose

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlesim/Pose';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Theta, info.Theta] = ros.internal.ros.messages.ros.default_type('single',1);
[data.LinearVelocity, info.LinearVelocity] = ros.internal.ros.messages.ros.default_type('single',1);
[data.AngularVelocity, info.AngularVelocity] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'turtlesim/Pose';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'theta';
info.MatPath{4} = 'linear_velocity';
info.MatPath{5} = 'angular_velocity';
