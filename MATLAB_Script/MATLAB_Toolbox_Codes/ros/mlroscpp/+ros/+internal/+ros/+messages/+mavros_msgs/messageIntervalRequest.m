function [data, info] = messageIntervalRequest
%MessageInterval gives an empty data for mavros_msgs/MessageIntervalRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/MessageIntervalRequest';
[data.MessageId, info.MessageId] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.MessageRate, info.MessageRate] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/MessageIntervalRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'message_id';
info.MatPath{2} = 'message_rate';
