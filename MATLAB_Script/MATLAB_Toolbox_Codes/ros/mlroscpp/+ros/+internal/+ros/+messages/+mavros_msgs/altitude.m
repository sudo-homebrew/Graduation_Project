function [data, info] = altitude
%Altitude gives an empty data for mavros_msgs/Altitude

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/Altitude';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Monotonic, info.Monotonic] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Amsl, info.Amsl] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Local, info.Local] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Relative, info.Relative] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Terrain, info.Terrain] = ros.internal.ros.messages.ros.default_type('single',1);
[data.BottomClearance, info.BottomClearance] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/Altitude';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'monotonic';
info.MatPath{8} = 'amsl';
info.MatPath{9} = 'local';
info.MatPath{10} = 'relative';
info.MatPath{11} = 'terrain';
info.MatPath{12} = 'bottom_clearance';
