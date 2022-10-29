function [data, info] = intParameter
%IntParameter gives an empty data for dynamic_reconfigure/IntParameter

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_reconfigure/IntParameter';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'dynamic_reconfigure/IntParameter';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'value';
