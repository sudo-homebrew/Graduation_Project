function [data, info] = changeTopicResponse
%ChangeTopic gives an empty data for jsk_topic_tools/ChangeTopicResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_topic_tools/ChangeTopicResponse';
info.MessageType = 'jsk_topic_tools/ChangeTopicResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
