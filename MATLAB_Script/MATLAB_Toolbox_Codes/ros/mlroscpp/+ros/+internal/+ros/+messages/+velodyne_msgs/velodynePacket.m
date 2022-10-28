function [data, info] = velodynePacket
%VelodynePacket gives an empty data for velodyne_msgs/VelodynePacket

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'velodyne_msgs/VelodynePacket';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',1206);
info.MessageType = 'velodyne_msgs/VelodynePacket';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'data';
