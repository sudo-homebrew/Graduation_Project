function [data, info] = setStringRequest
%SetString gives an empty data for cob_srvs/SetStringRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_srvs/SetStringRequest';
[data.Data, info.Data] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_srvs/SetStringRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
