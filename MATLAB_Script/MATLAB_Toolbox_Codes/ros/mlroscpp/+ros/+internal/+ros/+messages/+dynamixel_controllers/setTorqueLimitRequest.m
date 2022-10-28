function [data, info] = setTorqueLimitRequest
%SetTorqueLimit gives an empty data for dynamixel_controllers/SetTorqueLimitRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetTorqueLimitRequest';
[data.TorqueLimit, info.TorqueLimit] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'dynamixel_controllers/SetTorqueLimitRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'torque_limit';
