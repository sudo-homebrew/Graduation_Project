function [data, info] = typeDef
%TypeDef gives an empty data for rosapi/TypeDef

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/TypeDef';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Fieldnames, info.Fieldnames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Fieldtypes, info.Fieldtypes] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Fieldarraylen, info.Fieldarraylen] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.Examples, info.Examples] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rosapi/TypeDef';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'type';
info.MatPath{2} = 'fieldnames';
info.MatPath{3} = 'fieldtypes';
info.MatPath{4} = 'fieldarraylen';
info.MatPath{5} = 'examples';
