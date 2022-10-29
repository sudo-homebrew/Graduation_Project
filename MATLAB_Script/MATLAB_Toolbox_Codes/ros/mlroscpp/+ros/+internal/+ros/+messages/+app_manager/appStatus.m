function [data, info] = appStatus
%AppStatus gives an empty data for app_manager/AppStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/AppStatus';
[data.INFO, info.INFO] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.WARN, info.WARN] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'app_manager/AppStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'INFO';
info.MatPath{2} = 'WARN';
info.MatPath{3} = 'ERROR';
info.MatPath{4} = 'type';
info.MatPath{5} = 'status';
