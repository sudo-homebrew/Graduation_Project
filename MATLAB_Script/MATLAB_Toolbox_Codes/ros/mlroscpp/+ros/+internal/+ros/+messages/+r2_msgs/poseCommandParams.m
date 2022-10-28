function [data, info] = poseCommandParams
%PoseCommandParams gives an empty data for r2_msgs/PoseCommandParams

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/PoseCommandParams';
[data.MaxLinVel, info.MaxLinVel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MaxRotVel, info.MaxRotVel] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'r2_msgs/PoseCommandParams';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'maxLinVel';
info.MatPath{2} = 'maxRotVel';
