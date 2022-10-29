function [data, info] = uninstallAppResponse
%UninstallApp gives an empty data for app_manager/UninstallAppResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/UninstallAppResponse';
[data.Uninstalled, info.Uninstalled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'app_manager/UninstallAppResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'uninstalled';
info.MatPath{2} = 'message';
