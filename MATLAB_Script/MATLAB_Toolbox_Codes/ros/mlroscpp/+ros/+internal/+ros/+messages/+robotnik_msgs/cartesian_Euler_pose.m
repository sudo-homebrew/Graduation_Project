function [data, info] = cartesian_Euler_pose
%Cartesian_Euler_pose gives an empty data for robotnik_msgs/Cartesian_Euler_pose

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Cartesian_Euler_pose';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('double',1);
[data.A, info.A] = ros.internal.ros.messages.ros.default_type('double',1);
[data.B, info.B] = ros.internal.ros.messages.ros.default_type('double',1);
[data.C, info.C] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'robotnik_msgs/Cartesian_Euler_pose';
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
