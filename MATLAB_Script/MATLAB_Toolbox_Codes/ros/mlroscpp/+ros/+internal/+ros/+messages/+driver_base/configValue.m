function [data, info] = configValue
%ConfigValue gives an empty data for driver_base/ConfigValue

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'driver_base/ConfigValue';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'driver_base/ConfigValue';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'value';
