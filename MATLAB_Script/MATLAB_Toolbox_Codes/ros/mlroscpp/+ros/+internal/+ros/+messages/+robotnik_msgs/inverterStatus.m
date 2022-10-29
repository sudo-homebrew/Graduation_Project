function [data, info] = inverterStatus
%InverterStatus gives an empty data for robotnik_msgs/InverterStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/InverterStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.AcVoltage, info.AcVoltage] = ros.internal.ros.messages.ros.default_type('single',1);
[data.DcVoltage, info.DcVoltage] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Load, info.Load] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Percentage, info.Percentage] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Temperature, info.Temperature] = ros.internal.ros.messages.ros.default_type('single',1);
[data.On, info.On] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SerialNumber, info.SerialNumber] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/InverterStatus';
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
info.MatPath{7} = 'ac_voltage';
info.MatPath{8} = 'dc_voltage';
info.MatPath{9} = 'load';
info.MatPath{10} = 'percentage';
info.MatPath{11} = 'temperature';
info.MatPath{12} = 'on';
info.MatPath{13} = 'serial_number';
