function [data, info] = flush
%Flush gives an empty data for shared_serial/Flush

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/Flush';
[data.Socket, info.Socket] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'shared_serial/Flush';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'socket';
info.MatPath{2} = 'timeout';
