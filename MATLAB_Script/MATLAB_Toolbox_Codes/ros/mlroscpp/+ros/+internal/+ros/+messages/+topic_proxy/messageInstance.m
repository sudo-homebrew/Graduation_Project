function [data, info] = messageInstance
%MessageInstance gives an empty data for topic_proxy/MessageInstance

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/MessageInstance';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Md5sum, info.Md5sum] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.MessageDefinition, info.MessageDefinition] = ros.internal.ros.messages.ros.char('string',0);
[data.Blob, info.Blob] = ros.internal.ros.messages.blob.blob;
info.Blob.MLdataType = 'struct';
info.MessageType = 'topic_proxy/MessageInstance';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'md5sum';
info.MatPath{3} = 'type';
info.MatPath{4} = 'message_definition';
info.MatPath{5} = 'blob';
info.MatPath{6} = 'blob.compressed';
info.MatPath{7} = 'blob.data';
