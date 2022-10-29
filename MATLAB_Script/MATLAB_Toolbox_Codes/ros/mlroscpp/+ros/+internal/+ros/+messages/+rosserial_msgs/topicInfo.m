function [data, info] = topicInfo
%TopicInfo gives an empty data for rosserial_msgs/TopicInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosserial_msgs/TopicInfo';
[data.IDPUBLISHER, info.IDPUBLISHER] = ros.internal.ros.messages.ros.default_type('uint16',1, 0);
[data.IDSUBSCRIBER, info.IDSUBSCRIBER] = ros.internal.ros.messages.ros.default_type('uint16',1, 1);
[data.IDSERVICESERVER, info.IDSERVICESERVER] = ros.internal.ros.messages.ros.default_type('uint16',1, 2);
[data.IDSERVICECLIENT, info.IDSERVICECLIENT] = ros.internal.ros.messages.ros.default_type('uint16',1, 4);
[data.IDPARAMETERREQUEST, info.IDPARAMETERREQUEST] = ros.internal.ros.messages.ros.default_type('uint16',1, 6);
[data.IDLOG, info.IDLOG] = ros.internal.ros.messages.ros.default_type('uint16',1, 7);
[data.IDTIME, info.IDTIME] = ros.internal.ros.messages.ros.default_type('uint16',1, 10);
[data.TopicId, info.TopicId] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.TopicName, info.TopicName] = ros.internal.ros.messages.ros.char('string',0);
[data.MessageType_, info.MessageType_] = ros.internal.ros.messages.ros.char('string',0);
[data.Md5sum, info.Md5sum] = ros.internal.ros.messages.ros.char('string',0);
[data.BufferSize, info.BufferSize] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'rosserial_msgs/TopicInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'ID_PUBLISHER';
info.MatPath{2} = 'ID_SUBSCRIBER';
info.MatPath{3} = 'ID_SERVICE_SERVER';
info.MatPath{4} = 'ID_SERVICE_CLIENT';
info.MatPath{5} = 'ID_PARAMETER_REQUEST';
info.MatPath{6} = 'ID_LOG';
info.MatPath{7} = 'ID_TIME';
info.MatPath{8} = 'topic_id';
info.MatPath{9} = 'topic_name';
info.MatPath{10} = 'message_type';
info.MatPath{11} = 'md5sum';
info.MatPath{12} = 'buffer_size';
