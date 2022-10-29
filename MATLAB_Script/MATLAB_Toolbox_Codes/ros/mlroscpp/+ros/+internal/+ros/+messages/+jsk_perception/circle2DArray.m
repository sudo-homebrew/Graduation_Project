function [data, info] = circle2DArray
%Circle2DArray gives an empty data for jsk_perception/Circle2DArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/Circle2DArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Circles, info.Circles] = ros.internal.ros.messages.jsk_perception.circle2D;
info.Circles.MLdataType = 'struct';
info.Circles.MaxLen = NaN;
info.Circles.MinLen = 0;
data.Circles = data.Circles([],1);
info.MessageType = 'jsk_perception/Circle2DArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'circles';
info.MatPath{8} = 'circles.header';
info.MatPath{9} = 'circles.header.seq';
info.MatPath{10} = 'circles.header.stamp';
info.MatPath{11} = 'circles.header.stamp.sec';
info.MatPath{12} = 'circles.header.stamp.nsec';
info.MatPath{13} = 'circles.header.frame_id';
info.MatPath{14} = 'circles.radius';
info.MatPath{15} = 'circles.x';
info.MatPath{16} = 'circles.y';
