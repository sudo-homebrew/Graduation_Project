function [data, info] = sendStringRequest
%SendString gives an empty data for adhoc_communication/SendStringRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendStringRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Data, info.Data] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'adhoc_communication/SendStringRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'data';
