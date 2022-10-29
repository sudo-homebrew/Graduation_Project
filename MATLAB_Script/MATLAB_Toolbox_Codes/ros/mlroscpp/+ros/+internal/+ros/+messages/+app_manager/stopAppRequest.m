function [data, info] = stopAppRequest
%StopApp gives an empty data for app_manager/StopAppRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/StopAppRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'app_manager/StopAppRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'name';
