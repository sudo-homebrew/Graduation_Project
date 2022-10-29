function [data, info] = manualControl
%ManualControl gives an empty data for mavros_msgs/ManualControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ManualControl';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('single',1);
[data.R, info.R] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Buttons, info.Buttons] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'mavros_msgs/ManualControl';
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
info.MatPath{7} = 'x';
info.MatPath{8} = 'y';
info.MatPath{9} = 'z';
info.MatPath{10} = 'r';
info.MatPath{11} = 'buttons';
