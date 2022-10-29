function [data, info] = ptuSetVelResult
%PtuSetVelResult gives an empty data for ptu_control/PtuSetVelResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ptu_control/PtuSetVelResult';
[data.Pan, info.Pan] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Tilt, info.Tilt] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'ptu_control/PtuSetVelResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'pan';
info.MatPath{2} = 'tilt';
