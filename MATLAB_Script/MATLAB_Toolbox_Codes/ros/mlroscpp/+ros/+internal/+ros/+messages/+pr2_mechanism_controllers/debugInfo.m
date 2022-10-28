function [data, info] = debugInfo
%DebugInfo gives an empty data for pr2_mechanism_controllers/DebugInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/DebugInfo';
[data.Timing, info.Timing] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Sequence, info.Sequence] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.InputValid, info.InputValid] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Residual, info.Residual] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_mechanism_controllers/DebugInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'timing';
info.MatPath{2} = 'sequence';
info.MatPath{3} = 'input_valid';
info.MatPath{4} = 'residual';
