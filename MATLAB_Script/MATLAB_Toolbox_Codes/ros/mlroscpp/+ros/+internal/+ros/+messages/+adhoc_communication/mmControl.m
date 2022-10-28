function [data, info] = mmControl
%MmControl gives an empty data for adhoc_communication/MmControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/MmControl';
[data.SrcRobot, info.SrcRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.UpdateNumbers, info.UpdateNumbers] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'adhoc_communication/MmControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'src_robot';
info.MatPath{2} = 'update_numbers';
