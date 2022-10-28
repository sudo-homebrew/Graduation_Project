function [data, info] = motorHeadingOffset
%MotorHeadingOffset gives an empty data for robotnik_msgs/MotorHeadingOffset

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/MotorHeadingOffset';
[data.Motor, info.Motor] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'robotnik_msgs/MotorHeadingOffset';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'motor';
info.MatPath{2} = 'value';
