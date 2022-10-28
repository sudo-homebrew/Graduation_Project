function [data, info] = ackermannSetpt
%AckermannSetpt gives an empty data for clearpath_base/AckermannSetpt

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/AckermannSetpt';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Steering, info.Steering] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Throttle, info.Throttle] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Brake, info.Brake] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'clearpath_base/AckermannSetpt';
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
info.MatPath{7} = 'steering';
info.MatPath{8} = 'throttle';
info.MatPath{9} = 'brake';
