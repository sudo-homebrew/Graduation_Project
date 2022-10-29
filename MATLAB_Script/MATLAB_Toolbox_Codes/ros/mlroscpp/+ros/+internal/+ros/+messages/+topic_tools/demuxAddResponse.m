function [data, info] = demuxAddResponse
%DemuxAdd gives an empty data for topic_tools/DemuxAddResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_tools/DemuxAddResponse';
info.MessageType = 'topic_tools/DemuxAddResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
