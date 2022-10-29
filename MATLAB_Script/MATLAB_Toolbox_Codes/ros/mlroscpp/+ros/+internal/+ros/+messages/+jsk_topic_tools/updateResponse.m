function [data, info] = updateResponse
%Update gives an empty data for jsk_topic_tools/UpdateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/UpdateResponse';
[data.Rate, info.Rate] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_topic_tools/UpdateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'rate';
