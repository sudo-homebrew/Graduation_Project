function [data, info] = listResponse
%List gives an empty data for jsk_topic_tools/ListResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/ListResponse';
[data.TopicNames, info.TopicNames] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'jsk_topic_tools/ListResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'topic_names';
