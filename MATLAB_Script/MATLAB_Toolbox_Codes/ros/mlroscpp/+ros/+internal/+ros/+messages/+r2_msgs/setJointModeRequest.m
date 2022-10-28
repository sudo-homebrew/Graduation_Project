function [data, info] = setJointModeRequest
%SetJointMode gives an empty data for r2_msgs/SetJointModeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/SetJointModeRequest';
[data.ArmName, info.ArmName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'r2_msgs/SetJointModeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'arm_name';
