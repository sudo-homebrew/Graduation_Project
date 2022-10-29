function [data, info] = close
%Close gives an empty data for shared_serial/Close

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'shared_serial/Close';
[data.Socket, info.Socket] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'shared_serial/Close';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'socket';
