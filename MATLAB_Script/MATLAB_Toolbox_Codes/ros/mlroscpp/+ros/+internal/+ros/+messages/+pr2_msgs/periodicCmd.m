function [data, info] = periodicCmd
%PeriodicCmd gives an empty data for pr2_msgs/PeriodicCmd

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/PeriodicCmd';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Profile, info.Profile] = ros.internal.ros.messages.ros.char('string',0);
[data.Period, info.Period] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Amplitude, info.Amplitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Offset, info.Offset] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_msgs/PeriodicCmd';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'profile';
info.MatPath{8} = 'period';
info.MatPath{9} = 'amplitude';
info.MatPath{10} = 'offset';
