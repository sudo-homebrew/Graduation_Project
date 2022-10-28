function [data, info] = stopAppResponse
%StopApp gives an empty data for app_manager/StopAppResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/StopAppResponse';
[data.Stopped, info.Stopped] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ErrorCode, info.ErrorCode] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'app_manager/StopAppResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'stopped';
info.MatPath{2} = 'error_code';
info.MatPath{3} = 'message';
