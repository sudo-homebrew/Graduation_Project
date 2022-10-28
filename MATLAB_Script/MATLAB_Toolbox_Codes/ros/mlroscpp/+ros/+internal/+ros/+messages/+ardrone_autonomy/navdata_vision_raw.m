function [data, info] = navdata_vision_raw
%navdata_vision_raw gives an empty data for ardrone_autonomy/navdata_vision_raw

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/navdata_vision_raw';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.DroneTime, info.DroneTime] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Tag, info.Tag] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.VisionTxRaw, info.VisionTxRaw] = ros.internal.ros.messages.ros.default_type('single',1);
[data.VisionTyRaw, info.VisionTyRaw] = ros.internal.ros.messages.ros.default_type('single',1);
[data.VisionTzRaw, info.VisionTzRaw] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'ardrone_autonomy/navdata_vision_raw';
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
info.MatPath{7} = 'drone_time';
info.MatPath{8} = 'tag';
info.MatPath{9} = 'size';
info.MatPath{10} = 'vision_tx_raw';
info.MatPath{11} = 'vision_ty_raw';
info.MatPath{12} = 'vision_tz_raw';
