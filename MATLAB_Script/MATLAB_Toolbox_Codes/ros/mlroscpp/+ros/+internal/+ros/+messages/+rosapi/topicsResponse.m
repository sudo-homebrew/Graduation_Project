function [data, info] = topicsResponse
%Topics gives an empty data for rosapi/TopicsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/TopicsResponse';
[data.Topics, info.Topics] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/TopicsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'topics';
