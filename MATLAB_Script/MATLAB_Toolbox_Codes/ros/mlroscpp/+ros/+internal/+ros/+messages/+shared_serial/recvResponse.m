function [data, info] = recvResponse
%Recv gives an empty data for shared_serial/RecvResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/RecvResponse';
[data.Socket, info.Socket] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'shared_serial/RecvResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'socket';
info.MatPath{2} = 'data';
