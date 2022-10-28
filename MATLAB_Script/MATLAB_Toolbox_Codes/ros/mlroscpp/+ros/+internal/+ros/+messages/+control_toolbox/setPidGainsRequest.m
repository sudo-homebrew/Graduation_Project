function [data, info] = setPidGainsRequest
%SetPidGains gives an empty data for control_toolbox/SetPidGainsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_toolbox/SetPidGainsRequest';
[data.P, info.P] = ros.internal.ros.messages.ros.default_type('double',1);
[data.I, info.I] = ros.internal.ros.messages.ros.default_type('double',1);
[data.D, info.D] = ros.internal.ros.messages.ros.default_type('double',1);
[data.IClamp, info.IClamp] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Antiwindup, info.Antiwindup] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'control_toolbox/SetPidGainsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'p';
info.MatPath{2} = 'i';
info.MatPath{3} = 'd';
info.MatPath{4} = 'i_clamp';
info.MatPath{5} = 'antiwindup';
