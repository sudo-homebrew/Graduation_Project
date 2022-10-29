function [data, info] = content_msgResponse
%content_msg gives an empty data for iai_content_msgs/content_msgResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'iai_content_msgs/content_msgResponse';
[data.Content, info.Content] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'iai_content_msgs/content_msgResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'content';
