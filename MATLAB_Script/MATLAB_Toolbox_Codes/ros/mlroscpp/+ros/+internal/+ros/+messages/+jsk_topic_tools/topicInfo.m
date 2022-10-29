function [data, info] = topicInfo
%TopicInfo gives an empty data for jsk_topic_tools/TopicInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/TopicInfo';
[data.TopicName, info.TopicName] = ros.internal.ros.messages.ros.char('string',0);
[data.Rate, info.Rate] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_topic_tools/TopicInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'topic_name';
info.MatPath{2} = 'rate';
