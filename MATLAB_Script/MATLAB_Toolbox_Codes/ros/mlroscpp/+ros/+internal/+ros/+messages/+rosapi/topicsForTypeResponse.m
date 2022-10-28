function [data, info] = topicsForTypeResponse
%TopicsForType gives an empty data for rosapi/TopicsForTypeResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/TopicsForTypeResponse';
[data.Topics, info.Topics] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/TopicsForTypeResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'topics';
