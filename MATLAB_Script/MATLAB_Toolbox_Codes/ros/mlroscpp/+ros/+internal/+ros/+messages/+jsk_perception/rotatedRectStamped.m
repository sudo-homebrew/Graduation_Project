function [data, info] = rotatedRectStamped
%RotatedRectStamped gives an empty data for jsk_perception/RotatedRectStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/RotatedRectStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Rect, info.Rect] = ros.internal.ros.messages.jsk_perception.rotatedRect;
info.Rect.MLdataType = 'struct';
info.MessageType = 'jsk_perception/RotatedRectStamped';
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
info.MatPath{7} = 'rect';
info.MatPath{8} = 'rect.x';
info.MatPath{9} = 'rect.y';
info.MatPath{10} = 'rect.width';
info.MatPath{11} = 'rect.height';
info.MatPath{12} = 'rect.angle';
