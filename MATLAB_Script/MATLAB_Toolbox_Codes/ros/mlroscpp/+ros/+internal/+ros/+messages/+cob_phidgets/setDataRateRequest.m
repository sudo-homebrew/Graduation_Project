function [data, info] = setDataRateRequest
%SetDataRate gives an empty data for cob_phidgets/SetDataRateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_phidgets/SetDataRateRequest';
[data.Index, info.Index] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.DataRate, info.DataRate] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'cob_phidgets/SetDataRateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'index';
info.MatPath{2} = 'data_rate';
