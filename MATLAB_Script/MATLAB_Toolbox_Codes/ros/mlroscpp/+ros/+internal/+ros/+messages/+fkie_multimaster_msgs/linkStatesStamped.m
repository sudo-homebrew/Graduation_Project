function [data, info] = linkStatesStamped
%LinkStatesStamped gives an empty data for fkie_multimaster_msgs/LinkStatesStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/LinkStatesStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Links, info.Links] = ros.internal.ros.messages.fkie_multimaster_msgs.linkState;
info.Links.MLdataType = 'struct';
info.Links.MaxLen = NaN;
info.Links.MinLen = 0;
data.Links = data.Links([],1);
info.MessageType = 'fkie_multimaster_msgs/LinkStatesStamped';
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
info.MatPath{7} = 'links';
info.MatPath{8} = 'links.destination';
info.MatPath{9} = 'links.quality';
info.MatPath{10} = 'links.last_heartbeat';
info.MatPath{11} = 'links.last_heartbeat.sec';
info.MatPath{12} = 'links.last_heartbeat.nsec';
