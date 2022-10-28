function [data, info] = harkPower
%HarkPower gives an empty data for jsk_hark_msgs/HarkPower

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_hark_msgs/HarkPower';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Count, info.Count] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Directions, info.Directions] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.DataBytes, info.DataBytes] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Powers, info.Powers] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'jsk_hark_msgs/HarkPower';
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
info.MatPath{7} = 'count';
info.MatPath{8} = 'directions';
info.MatPath{9} = 'data_bytes';
info.MatPath{10} = 'powers';
