function [data, info] = setSpeedResponse
%SetSpeed gives an empty data for dynamixel_controllers/SetSpeedResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetSpeedResponse';
info.MessageType = 'dynamixel_controllers/SetSpeedResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
