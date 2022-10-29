function [data, info] = switchControllerResponse
%SwitchController gives an empty data for controller_manager_msgs/SwitchControllerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'controller_manager_msgs/SwitchControllerResponse';
[data.Ok, info.Ok] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'controller_manager_msgs/SwitchControllerResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'ok';
