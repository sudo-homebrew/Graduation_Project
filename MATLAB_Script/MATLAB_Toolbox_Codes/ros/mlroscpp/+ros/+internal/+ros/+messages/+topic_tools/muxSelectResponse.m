function [data, info] = muxSelectResponse
%MuxSelect gives an empty data for topic_tools/MuxSelectResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_tools/MuxSelectResponse';
[data.PrevTopic, info.PrevTopic] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'topic_tools/MuxSelectResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'prev_topic';
