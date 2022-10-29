function [data, info] = which_fingers_are_touchingRequest
%which_fingers_are_touching gives an empty data for sr_robot_msgs/which_fingers_are_touchingRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/which_fingers_are_touchingRequest';
[data.ForceThresholds, info.ForceThresholds] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'sr_robot_msgs/which_fingers_are_touchingRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'force_thresholds';
