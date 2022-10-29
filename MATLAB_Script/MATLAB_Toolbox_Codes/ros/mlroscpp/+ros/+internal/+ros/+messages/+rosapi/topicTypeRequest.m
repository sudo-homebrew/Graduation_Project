function [data, info] = topicTypeRequest
%TopicType gives an empty data for rosapi/TopicTypeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/TopicTypeRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rosapi/TopicTypeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'topic';
