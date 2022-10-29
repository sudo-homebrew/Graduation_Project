function [data, info] = sendRecvRequest
%SendRecv gives an empty data for shared_serial/SendRecvRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/SendRecvRequest';
[data.Socket, info.Socket] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.SendData, info.SendData] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
[data.Length, info.Length] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.RecvTimeout, info.RecvTimeout] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SockTimeout, info.SockTimeout] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'shared_serial/SendRecvRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'socket';
info.MatPath{2} = 'send_data';
info.MatPath{3} = 'length';
info.MatPath{4} = 'recv_timeout';
info.MatPath{5} = 'sock_timeout';
