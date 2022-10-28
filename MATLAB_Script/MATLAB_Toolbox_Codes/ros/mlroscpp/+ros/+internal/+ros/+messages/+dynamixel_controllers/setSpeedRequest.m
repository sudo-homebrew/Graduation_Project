function [data, info] = setSpeedRequest
%SetSpeed gives an empty data for dynamixel_controllers/SetSpeedRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetSpeedRequest';
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'dynamixel_controllers/SetSpeedRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'speed';
