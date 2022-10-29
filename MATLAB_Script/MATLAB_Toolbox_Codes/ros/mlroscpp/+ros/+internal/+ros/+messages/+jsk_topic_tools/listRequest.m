function [data, info] = listRequest
%List gives an empty data for jsk_topic_tools/ListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/ListRequest';
info.MessageType = 'jsk_topic_tools/ListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
