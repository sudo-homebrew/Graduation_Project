function [data, info] = connectRequest
%Connect gives an empty data for shared_serial/ConnectRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/ConnectRequest';
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'shared_serial/ConnectRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'timeout';
