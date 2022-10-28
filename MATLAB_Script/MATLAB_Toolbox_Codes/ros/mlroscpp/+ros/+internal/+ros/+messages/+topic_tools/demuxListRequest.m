function [data, info] = demuxListRequest
%DemuxList gives an empty data for topic_tools/DemuxListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_tools/DemuxListRequest';
info.MessageType = 'topic_tools/DemuxListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
