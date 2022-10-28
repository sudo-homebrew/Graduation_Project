function [data, info] = packet
%Packet gives an empty data for theora_image_transport/Packet

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'theora_image_transport/Packet';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
[data.BOS, info.BOS] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.EOS, info.EOS] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Granulepos, info.Granulepos] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Packetno, info.Packetno] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'theora_image_transport/Packet';
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
info.MatPath{7} = 'data';
info.MatPath{8} = 'b_o_s';
info.MatPath{9} = 'e_o_s';
info.MatPath{10} = 'granulepos';
info.MatPath{11} = 'packetno';
