function [data, info] = torqueEnableRequest
%TorqueEnable gives an empty data for dynamixel_controllers/TorqueEnableRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/TorqueEnableRequest';
[data.TorqueEnable, info.TorqueEnable] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'dynamixel_controllers/TorqueEnableRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'torque_enable';
