function [data, info] = pressureInfo
%PressureInfo gives an empty data for fingertip_pressure/PressureInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fingertip_pressure/PressureInfo';
[data.Sensor, info.Sensor] = ros.internal.ros.messages.fingertip_pressure.pressureInfoElement;
info.Sensor.MLdataType = 'struct';
info.Sensor.MaxLen = NaN;
info.Sensor.MinLen = 0;
data.Sensor = data.Sensor([],1);
info.MessageType = 'fingertip_pressure/PressureInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'sensor';
info.MatPath{2} = 'sensor.frame_id';
info.MatPath{3} = 'sensor.center';
info.MatPath{4} = 'sensor.center.x';
info.MatPath{5} = 'sensor.center.y';
info.MatPath{6} = 'sensor.center.z';
info.MatPath{7} = 'sensor.halfside1';
info.MatPath{8} = 'sensor.halfside1.x';
info.MatPath{9} = 'sensor.halfside1.y';
info.MatPath{10} = 'sensor.halfside1.z';
info.MatPath{11} = 'sensor.halfside2';
info.MatPath{12} = 'sensor.halfside2.x';
info.MatPath{13} = 'sensor.halfside2.y';
info.MatPath{14} = 'sensor.halfside2.z';
info.MatPath{15} = 'sensor.force_per_unit';
