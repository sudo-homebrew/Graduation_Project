function [data, info] = hasParamResponse
%HasParam gives an empty data for rosapi/HasParamResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/HasParamResponse';
[data.Exists, info.Exists] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rosapi/HasParamResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'exists';
