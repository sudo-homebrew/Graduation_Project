function [data, info] = differentialSpeed
%DifferentialSpeed gives an empty data for clearpath_base/DifferentialSpeed

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/DifferentialSpeed';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.LeftSpeed, info.LeftSpeed] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RightSpeed, info.RightSpeed] = ros.internal.ros.messages.ros.default_type('double',1);
[data.LeftAccel, info.LeftAccel] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RightAccel, info.RightAccel] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'clearpath_base/DifferentialSpeed';
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
info.MatPath{7} = 'left_speed';
info.MatPath{8} = 'right_speed';
info.MatPath{9} = 'left_accel';
info.MatPath{10} = 'right_accel';
