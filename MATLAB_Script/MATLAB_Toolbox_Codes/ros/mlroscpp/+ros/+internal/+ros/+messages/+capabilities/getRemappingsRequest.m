function [data, info] = getRemappingsRequest
%GetRemappings gives an empty data for capabilities/GetRemappingsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/GetRemappingsRequest';
[data.Spec, info.Spec] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/GetRemappingsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'spec';
