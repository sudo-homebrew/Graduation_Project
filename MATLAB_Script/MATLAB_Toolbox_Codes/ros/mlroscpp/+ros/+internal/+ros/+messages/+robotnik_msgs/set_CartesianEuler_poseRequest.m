function [data, info] = set_CartesianEuler_poseRequest
%set_CartesianEuler_pose gives an empty data for robotnik_msgs/set_CartesianEuler_poseRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_CartesianEuler_poseRequest';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('single',1);
[data.A, info.A] = ros.internal.ros.messages.ros.default_type('single',1);
[data.B, info.B] = ros.internal.ros.messages.ros.default_type('single',1);
[data.C, info.C] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'robotnik_msgs/set_CartesianEuler_poseRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'x';
info.MatPath{2} = 'y';
info.MatPath{3} = 'z';
info.MatPath{4} = 'A';
info.MatPath{5} = 'B';
info.MatPath{6} = 'C';
