function [data, info] = stopControllerRequest
%StopController gives an empty data for dynamixel_controllers/StopControllerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/StopControllerRequest';
[data.ControllerName, info.ControllerName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'dynamixel_controllers/StopControllerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'controller_name';
