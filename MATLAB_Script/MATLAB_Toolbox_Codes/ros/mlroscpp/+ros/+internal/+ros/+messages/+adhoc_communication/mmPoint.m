function [data, info] = mmPoint
%MmPoint gives an empty data for adhoc_communication/MmPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/MmPoint';
[data.SrcRobot, info.SrcRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'adhoc_communication/MmPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'src_robot';
info.MatPath{2} = 'x';
info.MatPath{3} = 'y';
