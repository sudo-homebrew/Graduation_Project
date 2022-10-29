function [data, info] = hectorDebugInfo
%HectorDebugInfo gives an empty data for hector_mapping/HectorDebugInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_mapping/HectorDebugInfo';
[data.IterData, info.IterData] = ros.internal.ros.messages.hector_mapping.hectorIterData;
info.IterData.MLdataType = 'struct';
info.IterData.MaxLen = NaN;
info.IterData.MinLen = 0;
data.IterData = data.IterData([],1);
info.MessageType = 'hector_mapping/HectorDebugInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'iterData';
info.MatPath{2} = 'iterData.hessian';
info.MatPath{3} = 'iterData.conditionNum';
info.MatPath{4} = 'iterData.determinant';
info.MatPath{5} = 'iterData.conditionNum2d';
info.MatPath{6} = 'iterData.determinant2d';
