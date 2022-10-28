function [data, info] = nodeletLoadRequest
%NodeletLoad gives an empty data for nodelet/NodeletLoadRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nodelet/NodeletLoadRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.RemapSourceArgs, info.RemapSourceArgs] = ros.internal.ros.messages.ros.char('string',NaN);
[data.RemapTargetArgs, info.RemapTargetArgs] = ros.internal.ros.messages.ros.char('string',NaN);
[data.MyArgv, info.MyArgv] = ros.internal.ros.messages.ros.char('string',NaN);
[data.BondId, info.BondId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'nodelet/NodeletLoadRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'name';
info.MatPath{2} = 'type';
info.MatPath{3} = 'remap_source_args';
info.MatPath{4} = 'remap_target_args';
info.MatPath{5} = 'my_argv';
info.MatPath{6} = 'bond_id';
