function [data, info] = publishMessageRequest
%PublishMessage gives an empty data for topic_proxy/PublishMessageRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/PublishMessageRequest';
[data.Message, info.Message] = ros.internal.ros.messages.topic_proxy.messageInstance;
info.Message.MLdataType = 'struct';
[data.Latch, info.Latch] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'topic_proxy/PublishMessageRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'message';
info.MatPath{2} = 'message.topic';
info.MatPath{3} = 'message.md5sum';
info.MatPath{4} = 'message.type';
info.MatPath{5} = 'message.message_definition';
info.MatPath{6} = 'message.blob';
info.MatPath{7} = 'message.blob.compressed';
info.MatPath{8} = 'message.blob.data';
info.MatPath{9} = 'latch';
