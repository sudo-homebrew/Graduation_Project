function [data, info] = turnSetpt
%TurnSetpt gives an empty data for clearpath_base/TurnSetpt

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/TurnSetpt';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.TransVel, info.TransVel] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TurnRadius, info.TurnRadius] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TransAccel, info.TransAccel] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'clearpath_base/TurnSetpt';
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
info.MatPath{7} = 'trans_vel';
info.MatPath{8} = 'turn_radius';
info.MatPath{9} = 'trans_accel';
