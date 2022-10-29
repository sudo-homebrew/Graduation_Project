function [data, info] = batteryState
%BatteryState gives an empty data for create_node/BatteryState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'create_node/BatteryState';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Temperature, info.Temperature] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Charge, info.Charge] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Capacity, info.Capacity] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'create_node/BatteryState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'temperature';
info.MatPath{8} = 'charge';
info.MatPath{9} = 'capacity';
