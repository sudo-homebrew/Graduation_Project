function [data, info] = mask
%Mask gives an empty data for cob_perception_msgs/Mask

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_perception_msgs/Mask';
[data.Roi, info.Roi] = ros.internal.ros.messages.cob_perception_msgs.rect;
info.Roi.MLdataType = 'struct';
[data.Mask_, info.Mask_] = ros.internal.ros.messages.sensor_msgs.image;
info.Mask_.MLdataType = 'struct';
info.MessageType = 'cob_perception_msgs/Mask';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'roi';
info.MatPath{2} = 'roi.x';
info.MatPath{3} = 'roi.y';
info.MatPath{4} = 'roi.width';
info.MatPath{5} = 'roi.height';
info.MatPath{6} = 'mask';
info.MatPath{7} = 'mask.header';
info.MatPath{8} = 'mask.header.seq';
info.MatPath{9} = 'mask.header.stamp';
info.MatPath{10} = 'mask.header.stamp.sec';
info.MatPath{11} = 'mask.header.stamp.nsec';
info.MatPath{12} = 'mask.header.frame_id';
info.MatPath{13} = 'mask.height';
info.MatPath{14} = 'mask.width';
info.MatPath{15} = 'mask.encoding';
info.MatPath{16} = 'mask.is_bigendian';
info.MatPath{17} = 'mask.step';
info.MatPath{18} = 'mask.data';
