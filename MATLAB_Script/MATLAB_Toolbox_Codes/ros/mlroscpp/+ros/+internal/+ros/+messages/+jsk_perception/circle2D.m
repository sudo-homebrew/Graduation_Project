function [data, info] = circle2D
%Circle2D gives an empty data for jsk_perception/Circle2D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/Circle2D';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Radius, info.Radius] = ros.internal.ros.messages.ros.default_type('double',1);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_perception/Circle2D';
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
info.MatPath{7} = 'radius';
info.MatPath{8} = 'x';
info.MatPath{9} = 'y';
