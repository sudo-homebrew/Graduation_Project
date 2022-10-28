function [data, info] = vFR_HUD
%VFR_HUD gives an empty data for mavros_msgs/VFR_HUD

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/VFR_HUD';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Airspeed, info.Airspeed] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Groundspeed, info.Groundspeed] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Heading, info.Heading] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.Throttle, info.Throttle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Altitude, info.Altitude] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Climb, info.Climb] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/VFR_HUD';
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
info.MatPath{7} = 'airspeed';
info.MatPath{8} = 'groundspeed';
info.MatPath{9} = 'heading';
info.MatPath{10} = 'throttle';
info.MatPath{11} = 'altitude';
info.MatPath{12} = 'climb';
