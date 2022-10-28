function [data, info] = uBI0
%UBI0 gives an empty data for sr_robot_msgs/UBI0

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/UBI0';
[data.Distal, info.Distal] = ros.internal.ros.messages.ros.default_type('uint16',12);
info.MessageType = 'sr_robot_msgs/UBI0';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'distal';
