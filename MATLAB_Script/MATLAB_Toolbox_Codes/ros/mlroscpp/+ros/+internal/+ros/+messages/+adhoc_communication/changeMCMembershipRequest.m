function [data, info] = changeMCMembershipRequest
%ChangeMCMembership gives an empty data for adhoc_communication/ChangeMCMembershipRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/ChangeMCMembershipRequest';
[data.GroupName, info.GroupName] = ros.internal.ros.messages.ros.char('string',0);
[data.Action, info.Action] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'adhoc_communication/ChangeMCMembershipRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'group_name';
info.MatPath{2} = 'action';
