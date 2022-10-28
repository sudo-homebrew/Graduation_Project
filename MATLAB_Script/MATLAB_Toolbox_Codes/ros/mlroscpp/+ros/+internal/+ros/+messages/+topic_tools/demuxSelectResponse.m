function [data, info] = demuxSelectResponse
%DemuxSelect gives an empty data for topic_tools/DemuxSelectResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_tools/DemuxSelectResponse';
[data.PrevTopic, info.PrevTopic] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'topic_tools/DemuxSelectResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'prev_topic';
