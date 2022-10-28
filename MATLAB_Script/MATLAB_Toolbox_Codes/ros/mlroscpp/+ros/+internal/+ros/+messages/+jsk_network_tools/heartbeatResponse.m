function [data, info] = heartbeatResponse
%HeartbeatResponse gives an empty data for jsk_network_tools/HeartbeatResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_network_tools/HeartbeatResponse';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Heartbeat, info.Heartbeat] = ros.internal.ros.messages.jsk_network_tools.heartbeat;
info.Heartbeat.MLdataType = 'struct';
info.MessageType = 'jsk_network_tools/HeartbeatResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'heartbeat';
info.MatPath{8} = 'heartbeat.header';
info.MatPath{9} = 'heartbeat.header.seq';
info.MatPath{10} = 'heartbeat.header.stamp';
info.MatPath{11} = 'heartbeat.header.stamp.sec';
info.MatPath{12} = 'heartbeat.header.stamp.nsec';
info.MatPath{13} = 'heartbeat.header.frame_id';
info.MatPath{14} = 'heartbeat.rate';
