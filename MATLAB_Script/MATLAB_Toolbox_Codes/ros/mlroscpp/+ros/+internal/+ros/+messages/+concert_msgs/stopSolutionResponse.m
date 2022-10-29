function [data, info] = stopSolutionResponse
%StopSolution gives an empty data for concert_msgs/StopSolutionResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/StopSolutionResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'concert_msgs/StopSolutionResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'message';
