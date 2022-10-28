function [data, info] = send
%Send gives an empty data for shared_serial/Send

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/Send';
[data.Socket, info.Socket] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'shared_serial/Send';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'socket';
info.MatPath{2} = 'data';
info.MatPath{3} = 'timeout';
