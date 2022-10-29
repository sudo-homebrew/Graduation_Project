function [data, info] = statusCodes
%StatusCodes gives an empty data for app_manager/StatusCodes

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/StatusCodes';
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.BADREQUEST, info.BADREQUEST] = ros.internal.ros.messages.ros.default_type('int32',1, 400);
[data.NOTFOUND, info.NOTFOUND] = ros.internal.ros.messages.ros.default_type('int32',1, 404);
[data.NOTRUNNING, info.NOTRUNNING] = ros.internal.ros.messages.ros.default_type('int32',1, 430);
[data.INTERNALERROR, info.INTERNALERROR] = ros.internal.ros.messages.ros.default_type('int32',1, 500);
[data.APPINVALID, info.APPINVALID] = ros.internal.ros.messages.ros.default_type('int32',1, 510);
[data.MULTIAPPNOTSUPPORTED, info.MULTIAPPNOTSUPPORTED] = ros.internal.ros.messages.ros.default_type('int32',1, 511);
info.MessageType = 'app_manager/StatusCodes';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'SUCCESS';
info.MatPath{2} = 'BAD_REQUEST';
info.MatPath{3} = 'NOT_FOUND';
info.MatPath{4} = 'NOT_RUNNING';
info.MatPath{5} = 'INTERNAL_ERROR';
info.MatPath{6} = 'APP_INVALID';
info.MatPath{7} = 'MULTIAPP_NOT_SUPPORTED';
