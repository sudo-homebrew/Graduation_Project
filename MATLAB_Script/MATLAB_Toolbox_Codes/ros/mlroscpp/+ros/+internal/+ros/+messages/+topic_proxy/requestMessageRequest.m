function [data, info] = requestMessageRequest
%RequestMessage gives an empty data for topic_proxy/RequestMessageRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/RequestMessageRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.RemoteTopic, info.RemoteTopic] = ros.internal.ros.messages.ros.char('string',0);
[data.Compressed, info.Compressed] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.duration;
info.Timeout.MLdataType = 'struct';
[data.Interval, info.Interval] = ros.internal.ros.messages.ros.duration;
info.Interval.MLdataType = 'struct';
[data.Latch, info.Latch] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'topic_proxy/RequestMessageRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'remote_topic';
info.MatPath{3} = 'compressed';
info.MatPath{4} = 'timeout';
info.MatPath{5} = 'timeout.sec';
info.MatPath{6} = 'timeout.nsec';
info.MatPath{7} = 'interval';
info.MatPath{8} = 'interval.sec';
info.MatPath{9} = 'interval.nsec';
info.MatPath{10} = 'latch';
