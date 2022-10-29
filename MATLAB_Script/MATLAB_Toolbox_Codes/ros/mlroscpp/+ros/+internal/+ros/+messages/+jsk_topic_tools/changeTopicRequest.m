function [data, info] = changeTopicRequest
%ChangeTopic gives an empty data for jsk_topic_tools/ChangeTopicRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/ChangeTopicRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_topic_tools/ChangeTopicRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'topic';
