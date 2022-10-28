function [data, info] = sendBytesRequest
%SendBytes gives an empty data for rosbridge_library/SendBytesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/SendBytesRequest';
[data.Count, info.Count] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'rosbridge_library/SendBytesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'count';
