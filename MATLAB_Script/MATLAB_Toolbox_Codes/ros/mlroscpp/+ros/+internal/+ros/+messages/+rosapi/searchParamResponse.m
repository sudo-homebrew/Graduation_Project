function [data, info] = searchParamResponse
%SearchParam gives an empty data for rosapi/SearchParamResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/SearchParamResponse';
[data.GlobalName, info.GlobalName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rosapi/SearchParamResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'global_name';
