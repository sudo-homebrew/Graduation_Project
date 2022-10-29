function [data, info] = capability
%Capability gives an empty data for capabilities/Capability

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/Capability';
[data.Capability_, info.Capability_] = ros.internal.ros.messages.ros.char('string',0);
[data.Provider, info.Provider] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/Capability';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'capability';
info.MatPath{2} = 'provider';
