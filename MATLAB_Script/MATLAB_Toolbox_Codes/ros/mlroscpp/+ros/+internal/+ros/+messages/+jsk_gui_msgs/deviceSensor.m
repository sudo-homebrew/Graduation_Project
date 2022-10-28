function [data, info] = deviceSensor
%DeviceSensor gives an empty data for jsk_gui_msgs/DeviceSensor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/DeviceSensor';
[data.Temperature, info.Temperature] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RelativeHumidity, info.RelativeHumidity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Light, info.Light] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Pressure, info.Pressure] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Proximity, info.Proximity] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_gui_msgs/DeviceSensor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'temperature';
info.MatPath{2} = 'relative_humidity';
info.MatPath{3} = 'light';
info.MatPath{4} = 'pressure';
info.MatPath{5} = 'proximity';
