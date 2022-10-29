function [data, info] = muxListRequest
%MuxList gives an empty data for topic_tools/MuxListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_tools/MuxListRequest';
info.MessageType = 'topic_tools/MuxListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
