function [data, info] = setDebugDataRequest
%SetDebugData gives an empty data for sr_robot_msgs/SetDebugDataRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/SetDebugDataRequest';
[data.MotorIndex, info.MotorIndex] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.MotorDataType, info.MotorDataType] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.PublisherIndex, info.PublisherIndex] = ros.internal.ros.messages.ros.default_type('int16',1);
info.MessageType = 'sr_robot_msgs/SetDebugDataRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'motor_index';
info.MatPath{2} = 'motor_data_type';
info.MatPath{3} = 'publisher_index';
