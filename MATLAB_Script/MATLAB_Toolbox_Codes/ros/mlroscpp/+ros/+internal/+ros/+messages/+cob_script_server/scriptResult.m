function [data, info] = scriptResult
%ScriptResult gives an empty data for cob_script_server/ScriptResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_script_server/ScriptResult';
[data.ErrorCode, info.ErrorCode] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'cob_script_server/ScriptResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'error_code';
