function [data, info] = setupChannelRequest
%SetupChannel gives an empty data for arbotix_msgs/SetupChannelRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'arbotix_msgs/SetupChannelRequest';
[data.TopicName, info.TopicName] = ros.internal.ros.messages.ros.char('string',0);
[data.Pin, info.Pin] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Rate, info.Rate] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'arbotix_msgs/SetupChannelRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'topic_name';
info.MatPath{2} = 'pin';
info.MatPath{3} = 'value';
info.MatPath{4} = 'rate';
