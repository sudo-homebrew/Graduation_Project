function [data, info] = fluidPressure
%FluidPressure gives an empty data for sensor_msgs/FluidPressure

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/FluidPressure';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.FluidPressure_, info.FluidPressure_] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Variance, info.Variance] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'sensor_msgs/FluidPressure';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'fluid_pressure';
info.MatPath{8} = 'variance';