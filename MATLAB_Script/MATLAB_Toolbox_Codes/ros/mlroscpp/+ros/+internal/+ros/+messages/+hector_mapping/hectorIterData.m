function [data, info] = hectorIterData
%HectorIterData gives an empty data for hector_mapping/HectorIterData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_mapping/HectorIterData';
[data.Hessian, info.Hessian] = ros.internal.ros.messages.ros.default_type('double',9);
[data.ConditionNum, info.ConditionNum] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Determinant, info.Determinant] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ConditionNum2d, info.ConditionNum2d] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Determinant2d, info.Determinant2d] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'hector_mapping/HectorIterData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'hessian';
info.MatPath{2} = 'conditionNum';
info.MatPath{3} = 'determinant';
info.MatPath{4} = 'conditionNum2d';
info.MatPath{5} = 'determinant2d';
