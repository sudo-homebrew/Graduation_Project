function [data, info] = updateRequest
%Update gives an empty data for jsk_topic_tools/UpdateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/UpdateRequest';
[data.TopicName, info.TopicName] = ros.internal.ros.messages.ros.char('string',0);
[data.Periodic, info.Periodic] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.PeriodicRate, info.PeriodicRate] = ros.internal.ros.messages.ros.duration;
info.PeriodicRate.MLdataType = 'struct';
info.MessageType = 'jsk_topic_tools/UpdateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'topic_name';
info.MatPath{2} = 'periodic';
info.MatPath{3} = 'periodic_rate';
info.MatPath{4} = 'periodic_rate.sec';
info.MatPath{5} = 'periodic_rate.nsec';
