function [data, info] = dockInfraRed
%DockInfraRed gives an empty data for kobuki_msgs/DockInfraRed

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/DockInfraRed';
[data.NEARLEFT, info.NEARLEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.NEARCENTER, info.NEARCENTER] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.NEARRIGHT, info.NEARRIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.FARLEFT, info.FARLEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 16);
[data.FARCENTER, info.FARCENTER] = ros.internal.ros.messages.ros.default_type('uint8',1, 8);
[data.FARRIGHT, info.FARRIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 32);
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'kobuki_msgs/DockInfraRed';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'NEAR_LEFT';
info.MatPath{2} = 'NEAR_CENTER';
info.MatPath{3} = 'NEAR_RIGHT';
info.MatPath{4} = 'FAR_LEFT';
info.MatPath{5} = 'FAR_CENTER';
info.MatPath{6} = 'FAR_RIGHT';
info.MatPath{7} = 'header';
info.MatPath{8} = 'header.seq';
info.MatPath{9} = 'header.stamp';
info.MatPath{10} = 'header.stamp.sec';
info.MatPath{11} = 'header.stamp.nsec';
info.MatPath{12} = 'header.frame_id';
info.MatPath{13} = 'data';
