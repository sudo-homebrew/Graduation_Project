function [data, info] = enable_disableResponse
%enable_disable gives an empty data for s3000_laser/enable_disableResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 's3000_laser/enable_disableResponse';
[data.Ret, info.Ret] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 's3000_laser/enable_disableResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'ret';