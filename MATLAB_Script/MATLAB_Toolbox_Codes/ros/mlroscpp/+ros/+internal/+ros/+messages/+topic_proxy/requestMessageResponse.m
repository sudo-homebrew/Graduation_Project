function [data, info] = requestMessageResponse
%RequestMessage gives an empty data for topic_proxy/RequestMessageResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/RequestMessageResponse';
info.MessageType = 'topic_proxy/RequestMessageResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
