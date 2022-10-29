function [data, info] = getMessageResponse
%GetMessage gives an empty data for topic_proxy/GetMessageResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/GetMessageResponse';
[data.Message, info.Message] = ros.internal.ros.messages.topic_proxy.messageInstance;
info.Message.MLdataType = 'struct';
info.MessageType = 'topic_proxy/GetMessageResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'message';
info.MatPath{2} = 'message.topic';
info.MatPath{3} = 'message.md5sum';
info.MatPath{4} = 'message.type';
info.MatPath{5} = 'message.message_definition';
info.MatPath{6} = 'message.blob';
info.MatPath{7} = 'message.blob.compressed';
info.MatPath{8} = 'message.blob.data';
