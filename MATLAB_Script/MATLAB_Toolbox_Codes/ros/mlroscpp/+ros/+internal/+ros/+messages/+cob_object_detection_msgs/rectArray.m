function [data, info] = rectArray
%RectArray gives an empty data for cob_object_detection_msgs/RectArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/RectArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Rects, info.Rects] = ros.internal.ros.messages.cob_object_detection_msgs.rect;
info.Rects.MLdataType = 'struct';
info.Rects.MaxLen = NaN;
info.Rects.MinLen = 0;
data.Rects = data.Rects([],1);
info.MessageType = 'cob_object_detection_msgs/RectArray';
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
info.MatPath{7} = 'rects';
info.MatPath{8} = 'rects.x';
info.MatPath{9} = 'rects.y';
info.MatPath{10} = 'rects.width';
info.MatPath{11} = 'rects.height';
