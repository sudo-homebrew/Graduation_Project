function [data, info] = restartControllerResponse
%RestartController gives an empty data for dynamixel_controllers/RestartControllerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/RestartControllerResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Reason, info.Reason] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'dynamixel_controllers/RestartControllerResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'reason';
