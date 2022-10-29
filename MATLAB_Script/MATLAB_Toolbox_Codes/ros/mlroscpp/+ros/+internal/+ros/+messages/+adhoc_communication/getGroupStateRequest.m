function [data, info] = getGroupStateRequest
%GetGroupState gives an empty data for adhoc_communication/GetGroupStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/GetGroupStateRequest';
[data.GroupName, info.GroupName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'adhoc_communication/GetGroupStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'group_name';
