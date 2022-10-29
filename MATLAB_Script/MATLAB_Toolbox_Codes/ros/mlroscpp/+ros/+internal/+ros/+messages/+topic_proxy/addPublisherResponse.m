function [data, info] = addPublisherResponse
%AddPublisher gives an empty data for topic_proxy/AddPublisherResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/AddPublisherResponse';
info.MessageType = 'topic_proxy/AddPublisherResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
