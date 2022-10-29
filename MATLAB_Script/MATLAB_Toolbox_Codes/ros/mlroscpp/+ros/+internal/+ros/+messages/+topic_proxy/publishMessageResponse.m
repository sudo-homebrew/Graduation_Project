function [data, info] = publishMessageResponse
%PublishMessage gives an empty data for topic_proxy/PublishMessageResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'topic_proxy/PublishMessageResponse';
info.MessageType = 'topic_proxy/PublishMessageResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
