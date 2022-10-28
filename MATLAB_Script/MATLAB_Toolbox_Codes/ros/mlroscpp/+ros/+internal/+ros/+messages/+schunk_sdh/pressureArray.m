function [data, info] = pressureArray
%PressureArray gives an empty data for schunk_sdh/PressureArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'schunk_sdh/PressureArray';
[data.SensorName, info.SensorName] = ros.internal.ros.messages.ros.char('string',0);
[data.CellsX, info.CellsX] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.CellsY, info.CellsY] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Pressure, info.Pressure] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'schunk_sdh/PressureArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'sensor_name';
info.MatPath{2} = 'cells_x';
info.MatPath{3} = 'cells_y';
info.MatPath{4} = 'pressure';
