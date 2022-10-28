function [data, info] = setIntRequest
%SetInt gives an empty data for cob_srvs/SetIntRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_srvs/SetIntRequest';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'cob_srvs/SetIntRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
