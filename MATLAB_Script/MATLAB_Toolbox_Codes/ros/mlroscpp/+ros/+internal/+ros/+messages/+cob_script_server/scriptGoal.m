function [data, info] = scriptGoal
%ScriptGoal gives an empty data for cob_script_server/ScriptGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_script_server/ScriptGoal';
[data.FunctionName, info.FunctionName] = ros.internal.ros.messages.ros.char('string',0);
[data.ComponentName, info.ComponentName] = ros.internal.ros.messages.ros.char('string',0);
[data.ParameterName, info.ParameterName] = ros.internal.ros.messages.ros.char('string',0);
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.char('string',0);
[data.ServiceName, info.ServiceName] = ros.internal.ros.messages.ros.char('string',0);
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Planning, info.Planning] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_script_server/ScriptGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'function_name';
info.MatPath{2} = 'component_name';
info.MatPath{3} = 'parameter_name';
info.MatPath{4} = 'mode';
info.MatPath{5} = 'service_name';
info.MatPath{6} = 'duration';
info.MatPath{7} = 'planning';
