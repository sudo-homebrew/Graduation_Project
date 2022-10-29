function [data, info] = paramDescription
%ParamDescription gives an empty data for dynamic_reconfigure/ParamDescription

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_reconfigure/ParamDescription';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Level, info.Level] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Description, info.Description] = ros.internal.ros.messages.ros.char('string',0);
[data.EditMethod, info.EditMethod] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'dynamic_reconfigure/ParamDescription';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'name';
info.MatPath{2} = 'type';
info.MatPath{3} = 'level';
info.MatPath{4} = 'description';
info.MatPath{5} = 'edit_method';
