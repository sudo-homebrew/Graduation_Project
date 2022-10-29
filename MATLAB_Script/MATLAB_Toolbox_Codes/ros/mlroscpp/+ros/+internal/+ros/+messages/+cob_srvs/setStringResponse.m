function [data, info] = setStringResponse
%SetString gives an empty data for cob_srvs/SetStringResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_srvs/SetStringResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_srvs/SetStringResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'message';
