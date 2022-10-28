function [data, info] = group
%Group gives an empty data for dynamic_reconfigure/Group

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_reconfigure/Group';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Parameters, info.Parameters] = ros.internal.ros.messages.dynamic_reconfigure.paramDescription;
info.Parameters.MLdataType = 'struct';
info.Parameters.MaxLen = NaN;
info.Parameters.MinLen = 0;
data.Parameters = data.Parameters([],1);
[data.Parent, info.Parent] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'dynamic_reconfigure/Group';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'name';
info.MatPath{2} = 'type';
info.MatPath{3} = 'parameters';
info.MatPath{4} = 'parameters.name';
info.MatPath{5} = 'parameters.type';
info.MatPath{6} = 'parameters.level';
info.MatPath{7} = 'parameters.description';
info.MatPath{8} = 'parameters.edit_method';
info.MatPath{9} = 'parent';
info.MatPath{10} = 'id';
