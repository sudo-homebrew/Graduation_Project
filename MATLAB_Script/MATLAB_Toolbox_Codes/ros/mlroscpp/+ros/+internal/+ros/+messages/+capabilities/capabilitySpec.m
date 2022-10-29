function [data, info] = capabilitySpec
%CapabilitySpec gives an empty data for capabilities/CapabilitySpec

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/CapabilitySpec';
[data.Package, info.Package] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Content, info.Content] = ros.internal.ros.messages.ros.char('string',0);
[data.DefaultProvider, info.DefaultProvider] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/CapabilitySpec';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'package';
info.MatPath{2} = 'type';
info.MatPath{3} = 'content';
info.MatPath{4} = 'default_provider';
