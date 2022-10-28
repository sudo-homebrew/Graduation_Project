function [data, info] = recvString
%RecvString gives an empty data for adhoc_communication/RecvString

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/RecvString';
[data.SrcRobot, info.SrcRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Data, info.Data] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'adhoc_communication/RecvString';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'src_robot';
info.MatPath{2} = 'data';
