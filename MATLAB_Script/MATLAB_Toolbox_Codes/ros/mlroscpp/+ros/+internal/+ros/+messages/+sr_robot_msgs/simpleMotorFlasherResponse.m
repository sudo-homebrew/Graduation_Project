function [data, info] = simpleMotorFlasherResponse
%SimpleMotorFlasher gives an empty data for sr_robot_msgs/SimpleMotorFlasherResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/SimpleMotorFlasherResponse';
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.FAIL, info.FAIL] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
info.MessageType = 'sr_robot_msgs/SimpleMotorFlasherResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'value';
info.MatPath{2} = 'SUCCESS';
info.MatPath{3} = 'FAIL';
