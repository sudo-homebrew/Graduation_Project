function [data, info] = stateResult
%StateResult gives an empty data for cob_script_server/StateResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_script_server/StateResult';
[data.ReturnValue, info.ReturnValue] = ros.internal.ros.messages.ros.default_type('int16',1);
info.MessageType = 'cob_script_server/StateResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'return_value';
