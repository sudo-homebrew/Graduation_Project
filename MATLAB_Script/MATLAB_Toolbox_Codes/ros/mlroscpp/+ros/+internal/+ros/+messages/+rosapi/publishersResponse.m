function [data, info] = publishersResponse
%Publishers gives an empty data for rosapi/PublishersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/PublishersResponse';
[data.Publishers, info.Publishers] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/PublishersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'publishers';
