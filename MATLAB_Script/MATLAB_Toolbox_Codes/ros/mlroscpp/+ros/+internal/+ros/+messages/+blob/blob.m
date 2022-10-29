function [data, info] = blob
%Blob gives an empty data for blob/Blob

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'blob/Blob';
[data.Compressed, info.Compressed] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'blob/Blob';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'compressed';
info.MatPath{2} = 'data';
