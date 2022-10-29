function [data, info] = getGroupStateResponse
%GetGroupState gives an empty data for adhoc_communication/GetGroupStateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/GetGroupStateResponse';
[data.Member, info.Member] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Activated, info.Activated] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Connected, info.Connected] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Root, info.Root] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Joining, info.Joining] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.RouteUplink, info.RouteUplink] = ros.internal.ros.messages.ros.char('string',0);
[data.Downlinks, info.Downlinks] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'adhoc_communication/GetGroupStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'member';
info.MatPath{2} = 'activated';
info.MatPath{3} = 'connected';
info.MatPath{4} = 'root';
info.MatPath{5} = 'joining';
info.MatPath{6} = 'route_uplink';
info.MatPath{7} = 'downlinks';
