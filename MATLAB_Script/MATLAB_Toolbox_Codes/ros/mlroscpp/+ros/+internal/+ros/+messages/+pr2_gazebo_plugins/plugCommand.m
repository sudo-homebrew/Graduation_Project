function [data, info] = plugCommand
%PlugCommand gives an empty data for pr2_gazebo_plugins/PlugCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gazebo_plugins/PlugCommand';
[data.AcPresent, info.AcPresent] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ChargeRate, info.ChargeRate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.DischargeRate, info.DischargeRate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Charge, info.Charge] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_gazebo_plugins/PlugCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'ac_present';
info.MatPath{2} = 'charge_rate';
info.MatPath{3} = 'discharge_rate';
info.MatPath{4} = 'charge';
