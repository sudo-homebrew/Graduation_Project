function [data, info] = changeMotorSystemControlsResponse
%ChangeMotorSystemControls gives an empty data for sr_robot_msgs/ChangeMotorSystemControlsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ChangeMotorSystemControlsResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.MOTORIDOUTOFRANGE, info.MOTORIDOUTOFRANGE] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
info.MessageType = 'sr_robot_msgs/ChangeMotorSystemControlsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'result';
info.MatPath{2} = 'SUCCESS';
info.MatPath{3} = 'MOTOR_ID_OUT_OF_RANGE';
