function [data, info] = reverseKinematics
%reverseKinematics gives an empty data for sr_robot_msgs/reverseKinematics

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/reverseKinematics';
[data.FingerName, info.FingerName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'sr_robot_msgs/reverseKinematics';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'finger_name';
