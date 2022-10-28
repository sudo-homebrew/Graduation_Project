function [data, info] = getParamResponse
%GetParam gives an empty data for mongodb_store/GetParamResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mongodb_store/GetParamResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ParamValue, info.ParamValue] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'mongodb_store/GetParamResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'param_value';
