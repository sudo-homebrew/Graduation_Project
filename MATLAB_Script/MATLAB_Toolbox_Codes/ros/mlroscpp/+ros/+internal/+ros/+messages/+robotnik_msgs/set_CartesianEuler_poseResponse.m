function [data, info] = set_CartesianEuler_poseResponse
%set_CartesianEuler_pose gives an empty data for robotnik_msgs/set_CartesianEuler_poseResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_CartesianEuler_poseResponse';
[data.Ret, info.Ret] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/set_CartesianEuler_poseResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'ret';
