function [data, info] = demuxListResponse
%DemuxList gives an empty data for topic_tools/DemuxListResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_tools/DemuxListResponse';
[data.Topics, info.Topics] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'topic_tools/DemuxListResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'topics';
