function [data, info] = changeControlTypeRequest
%ChangeControlType gives an empty data for sr_robot_msgs/ChangeControlTypeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ChangeControlTypeRequest';
[data.ControlType, info.ControlType] = ros.internal.ros.messages.sr_robot_msgs.controlType;
info.ControlType.MLdataType = 'struct';
info.MessageType = 'sr_robot_msgs/ChangeControlTypeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'control_type';
info.MatPath{2} = 'control_type.control_type';
info.MatPath{3} = 'control_type.PWM';
info.MatPath{4} = 'control_type.FORCE';
info.MatPath{5} = 'control_type.QUERY';
