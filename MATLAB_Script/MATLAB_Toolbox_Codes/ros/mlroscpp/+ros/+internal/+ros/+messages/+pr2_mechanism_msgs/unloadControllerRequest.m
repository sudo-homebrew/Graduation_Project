function [data, info] = unloadControllerRequest
%UnloadController gives an empty data for pr2_mechanism_msgs/UnloadControllerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_msgs/UnloadControllerRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_mechanism_msgs/UnloadControllerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'name';
