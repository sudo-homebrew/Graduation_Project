function [data, info] = emptyStringResponse
%EmptyString gives an empty data for rocon_std_msgs/EmptyStringResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/EmptyStringResponse';
[data.Data, info.Data] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rocon_std_msgs/EmptyStringResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
