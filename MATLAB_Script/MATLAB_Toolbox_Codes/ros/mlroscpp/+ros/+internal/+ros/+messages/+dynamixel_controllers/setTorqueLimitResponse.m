function [data, info] = setTorqueLimitResponse
%SetTorqueLimit gives an empty data for dynamixel_controllers/SetTorqueLimitResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetTorqueLimitResponse';
info.MessageType = 'dynamixel_controllers/SetTorqueLimitResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
