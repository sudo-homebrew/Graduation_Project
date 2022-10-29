function [data, info] = controlType
%ControlType gives an empty data for sr_robot_msgs/ControlType

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ControlType';
[data.ControlType_, info.ControlType_] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.PWM, info.PWM] = ros.internal.ros.messages.ros.default_type('int16',1, 0);
[data.FORCE, info.FORCE] = ros.internal.ros.messages.ros.default_type('int16',1, 1);
[data.QUERY, info.QUERY] = ros.internal.ros.messages.ros.default_type('int16',1, -1);
info.MessageType = 'sr_robot_msgs/ControlType';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'control_type';
info.MatPath{2} = 'PWM';
info.MatPath{3} = 'FORCE';
info.MatPath{4} = 'QUERY';
