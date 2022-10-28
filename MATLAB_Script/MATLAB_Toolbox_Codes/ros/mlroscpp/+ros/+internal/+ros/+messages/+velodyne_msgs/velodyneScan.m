function [data, info] = velodyneScan
%VelodyneScan gives an empty data for velodyne_msgs/VelodyneScan

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'velodyne_msgs/VelodyneScan';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Packets, info.Packets] = ros.internal.ros.messages.velodyne_msgs.velodynePacket;
info.Packets.MLdataType = 'struct';
info.Packets.MaxLen = NaN;
info.Packets.MinLen = 0;
data.Packets = data.Packets([],1);
info.MessageType = 'velodyne_msgs/VelodyneScan';
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
info.MatPath{7} = 'packets';
info.MatPath{8} = 'packets.stamp';
info.MatPath{9} = 'packets.stamp.sec';
info.MatPath{10} = 'packets.stamp.nsec';
info.MatPath{11} = 'packets.data';
