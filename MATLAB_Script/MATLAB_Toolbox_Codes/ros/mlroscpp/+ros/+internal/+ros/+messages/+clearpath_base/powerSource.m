function [data, info] = powerSource
%PowerSource gives an empty data for clearpath_base/PowerSource

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/PowerSource';
[data.Charge, info.Charge] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Capacity, info.Capacity] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Present, info.Present] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.InUse, info.InUse] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Description, info.Description] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'clearpath_base/PowerSource';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'charge';
info.MatPath{2} = 'capacity';
info.MatPath{3} = 'present';
info.MatPath{4} = 'in_use';
info.MatPath{5} = 'description';
