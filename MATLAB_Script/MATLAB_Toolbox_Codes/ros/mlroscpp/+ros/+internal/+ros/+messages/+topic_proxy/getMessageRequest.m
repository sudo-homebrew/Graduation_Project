function [data, info] = getMessageRequest
%GetMessage gives an empty data for topic_proxy/GetMessageRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/GetMessageRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Compressed, info.Compressed] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.duration;
info.Timeout.MLdataType = 'struct';
info.MessageType = 'topic_proxy/GetMessageRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'compressed';
info.MatPath{3} = 'timeout';
info.MatPath{4} = 'timeout.sec';
info.MatPath{5} = 'timeout.nsec';
