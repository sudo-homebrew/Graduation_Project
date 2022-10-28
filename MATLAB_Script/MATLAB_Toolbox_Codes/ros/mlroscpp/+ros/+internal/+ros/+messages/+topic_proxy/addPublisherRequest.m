function [data, info] = addPublisherRequest
%AddPublisher gives an empty data for topic_proxy/AddPublisherRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/AddPublisherRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.RemoteTopic, info.RemoteTopic] = ros.internal.ros.messages.ros.char('string',0);
[data.Compressed, info.Compressed] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Latch, info.Latch] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'topic_proxy/AddPublisherRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'remote_topic';
info.MatPath{3} = 'compressed';
info.MatPath{4} = 'latch';
