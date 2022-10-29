function [data, info] = mmListOfPoints
%MmListOfPoints gives an empty data for adhoc_communication/MmListOfPoints

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/MmListOfPoints';
[data.Positions, info.Positions] = ros.internal.ros.messages.adhoc_communication.mmPoint;
info.Positions.MLdataType = 'struct';
info.Positions.MaxLen = NaN;
info.Positions.MinLen = 0;
data.Positions = data.Positions([],1);
info.MessageType = 'adhoc_communication/MmListOfPoints';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'positions';
info.MatPath{2} = 'positions.src_robot';
info.MatPath{3} = 'positions.x';
info.MatPath{4} = 'positions.y';
