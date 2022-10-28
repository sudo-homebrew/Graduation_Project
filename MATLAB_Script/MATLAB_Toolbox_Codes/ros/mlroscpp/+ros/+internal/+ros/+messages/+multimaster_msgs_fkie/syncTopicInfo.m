function [data, info] = syncTopicInfo
%SyncTopicInfo gives an empty data for multimaster_msgs_fkie/SyncTopicInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/SyncTopicInfo';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Node, info.Node] = ros.internal.ros.messages.ros.char('string',0);
[data.Nodeuri, info.Nodeuri] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'multimaster_msgs_fkie/SyncTopicInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'node';
info.MatPath{3} = 'nodeuri';
