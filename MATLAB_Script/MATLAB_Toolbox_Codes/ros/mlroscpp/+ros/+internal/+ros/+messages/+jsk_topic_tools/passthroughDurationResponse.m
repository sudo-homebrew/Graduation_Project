function [data, info] = passthroughDurationResponse
%PassthroughDuration gives an empty data for jsk_topic_tools/PassthroughDurationResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/PassthroughDurationResponse';
info.MessageType = 'jsk_topic_tools/PassthroughDurationResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
