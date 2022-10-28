function [data, info] = motorTemperature
%MotorTemperature gives an empty data for ethercat_hardware/MotorTemperature

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_hardware/MotorTemperature';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.WindingTemperature, info.WindingTemperature] = ros.internal.ros.messages.ros.default_type('double',1);
[data.HousingTemperature, info.HousingTemperature] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AmbientTemperature, info.AmbientTemperature] = ros.internal.ros.messages.ros.default_type('double',1);
[data.HeatingPower, info.HeatingPower] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'ethercat_hardware/MotorTemperature';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'winding_temperature';
info.MatPath{5} = 'housing_temperature';
info.MatPath{6} = 'ambient_temperature';
info.MatPath{7} = 'heating_power';
