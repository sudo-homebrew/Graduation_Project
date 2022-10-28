function [data, info] = navdata_pressure_raw
%navdata_pressure_raw gives an empty data for ardrone_autonomy/navdata_pressure_raw

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/navdata_pressure_raw';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.DroneTime, info.DroneTime] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Tag, info.Tag] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Up, info.Up] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Ut, info.Ut] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.TemperatureMeas, info.TemperatureMeas] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.PressionMeas, info.PressionMeas] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'ardrone_autonomy/navdata_pressure_raw';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'drone_time';
info.MatPath{8} = 'tag';
info.MatPath{9} = 'size';
info.MatPath{10} = 'up';
info.MatPath{11} = 'ut';
info.MatPath{12} = 'Temperature_meas';
info.MatPath{13} = 'Pression_meas';
