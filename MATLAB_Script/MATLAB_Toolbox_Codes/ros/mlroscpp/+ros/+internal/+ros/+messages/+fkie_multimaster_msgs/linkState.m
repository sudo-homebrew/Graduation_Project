function [data, info] = linkState
%LinkState gives an empty data for fkie_multimaster_msgs/LinkState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'fkie_multimaster_msgs/LinkState';
[data.Destination, info.Destination] = ros.internal.ros.messages.ros.char('string',0);
[data.Quality, info.Quality] = ros.internal.ros.messages.ros.default_type('single',1);
[data.LastHeartbeat, info.LastHeartbeat] = ros.internal.ros.messages.ros.time;
info.LastHeartbeat.MLdataType = 'struct';
info.MessageType = 'fkie_multimaster_msgs/LinkState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'destination';
info.MatPath{2} = 'quality';
info.MatPath{3} = 'last_heartbeat';
info.MatPath{4} = 'last_heartbeat.sec';
info.MatPath{5} = 'last_heartbeat.nsec';
