function [data, info] = recvRequest
%Recv gives an empty data for shared_serial/RecvRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/RecvRequest';
[data.Socket, info.Socket] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Length, info.Length] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.RecvTimeout, info.RecvTimeout] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SockTimeout, info.SockTimeout] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'shared_serial/RecvRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'socket';
info.MatPath{2} = 'length';
info.MatPath{3} = 'recv_timeout';
info.MatPath{4} = 'sock_timeout';
