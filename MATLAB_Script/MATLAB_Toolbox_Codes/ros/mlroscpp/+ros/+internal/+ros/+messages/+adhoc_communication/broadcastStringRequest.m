function [data, info] = broadcastStringRequest
%BroadcastString gives an empty data for adhoc_communication/BroadcastStringRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/BroadcastStringRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Data, info.Data] = ros.internal.ros.messages.ros.char('string',0);
[data.HopLimit, info.HopLimit] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'adhoc_communication/BroadcastStringRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'data';
info.MatPath{3} = 'hop_limit';
