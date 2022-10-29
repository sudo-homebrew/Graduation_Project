function [data, info] = velocitySetpt
%VelocitySetpt gives an empty data for clearpath_base/VelocitySetpt

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/VelocitySetpt';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Trans, info.Trans] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Rot, info.Rot] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Accel, info.Accel] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'clearpath_base/VelocitySetpt';
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
info.MatPath{7} = 'trans';
info.MatPath{8} = 'rot';
info.MatPath{9} = 'accel';
