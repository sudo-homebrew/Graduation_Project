function [data, info] = powerState
%PowerState gives an empty data for pr2_msgs/PowerState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/PowerState';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.PowerConsumption, info.PowerConsumption] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TimeRemaining, info.TimeRemaining] = ros.internal.ros.messages.ros.duration;
info.TimeRemaining.MLdataType = 'struct';
[data.PredictionMethod, info.PredictionMethod] = ros.internal.ros.messages.ros.char('string',0);
[data.RelativeCapacity, info.RelativeCapacity] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.ACPresent, info.ACPresent] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'pr2_msgs/PowerState';
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
info.MatPath{7} = 'power_consumption';
info.MatPath{8} = 'time_remaining';
info.MatPath{9} = 'time_remaining.sec';
info.MatPath{10} = 'time_remaining.nsec';
info.MatPath{11} = 'prediction_method';
info.MatPath{12} = 'relative_capacity';
info.MatPath{13} = 'AC_present';
