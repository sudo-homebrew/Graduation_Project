function [data, info] = setPidGainsResponse
%SetPidGains gives an empty data for control_toolbox/SetPidGainsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_toolbox/SetPidGainsResponse';
info.MessageType = 'control_toolbox/SetPidGainsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
