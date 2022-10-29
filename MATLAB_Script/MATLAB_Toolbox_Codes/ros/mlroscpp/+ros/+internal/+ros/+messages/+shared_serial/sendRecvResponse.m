function [data, info] = sendRecvResponse
%SendRecv gives an empty data for shared_serial/SendRecvResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/SendRecvResponse';
[data.Socket, info.Socket] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.RecvData, info.RecvData] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'shared_serial/SendRecvResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'socket';
info.MatPath{2} = 'recv_data';
