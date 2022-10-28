function [data, info] = setBoolRequest
%SetBool gives an empty data for std_srvs/SetBoolRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_srvs/SetBoolRequest';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'std_srvs/SetBoolRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
