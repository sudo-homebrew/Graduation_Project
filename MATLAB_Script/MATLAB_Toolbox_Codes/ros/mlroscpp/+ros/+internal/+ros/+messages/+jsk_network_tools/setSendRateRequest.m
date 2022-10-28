function [data, info] = setSendRateRequest
%SetSendRate gives an empty data for jsk_network_tools/SetSendRateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_network_tools/SetSendRateRequest';
[data.Rate, info.Rate] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'jsk_network_tools/SetSendRateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'rate';
