function [data, info] = motorStatus
%MotorStatus gives an empty data for hector_uav_msgs/MotorStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_uav_msgs/MotorStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.On, info.On] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Running, info.Running] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Voltage, info.Voltage] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Frequency, info.Frequency] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Current, info.Current] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'hector_uav_msgs/MotorStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'on';
info.MatPath{8} = 'running';
info.MatPath{9} = 'voltage';
info.MatPath{10} = 'frequency';
info.MatPath{11} = 'current';
