function [data, info] = setFloatRequest
%SetFloat gives an empty data for cob_srvs/SetFloatRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_srvs/SetFloatRequest';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'cob_srvs/SetFloatRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
