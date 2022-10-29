function [data, info] = hasParamRequest
%HasParam gives an empty data for rosapi/HasParamRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/HasParamRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rosapi/HasParamRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'name';
