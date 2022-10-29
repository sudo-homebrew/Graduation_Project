function [data, info] = changeControlTypeResponse
%ChangeControlType gives an empty data for sr_robot_msgs/ChangeControlTypeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ChangeControlTypeResponse';
[data.Result, info.Result] = ros.internal.ros.messages.sr_robot_msgs.controlType;
info.Result.MLdataType = 'struct';
info.MessageType = 'sr_robot_msgs/ChangeControlTypeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'result';
info.MatPath{2} = 'result.control_type';
info.MatPath{3} = 'result.PWM';
info.MatPath{4} = 'result.FORCE';
info.MatPath{5} = 'result.QUERY';
