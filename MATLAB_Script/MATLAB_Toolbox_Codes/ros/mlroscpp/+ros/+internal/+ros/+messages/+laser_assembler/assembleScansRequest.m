function [data, info] = assembleScansRequest
%AssembleScans gives an empty data for laser_assembler/AssembleScansRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'laser_assembler/AssembleScansRequest';
[data.Begin, info.Begin] = ros.internal.ros.messages.ros.time;
info.Begin.MLdataType = 'struct';
[data.End, info.End] = ros.internal.ros.messages.ros.time;
info.End.MLdataType = 'struct';
info.MessageType = 'laser_assembler/AssembleScansRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'begin';
info.MatPath{2} = 'begin.sec';
info.MatPath{3} = 'begin.nsec';
info.MatPath{4} = 'end';
info.MatPath{5} = 'end.sec';
info.MatPath{6} = 'end.nsec';
