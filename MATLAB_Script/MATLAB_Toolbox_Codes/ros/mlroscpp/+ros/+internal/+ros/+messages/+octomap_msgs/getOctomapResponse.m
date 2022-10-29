function [data, info] = getOctomapResponse
%GetOctomap gives an empty data for octomap_msgs/GetOctomapResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'octomap_msgs/GetOctomapResponse';
[data.Map, info.Map] = ros.internal.ros.messages.octomap_msgs.octomap;
info.Map.MLdataType = 'struct';
info.MessageType = 'octomap_msgs/GetOctomapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'map';
info.MatPath{2} = 'map.header';
info.MatPath{3} = 'map.header.seq';
info.MatPath{4} = 'map.header.stamp';
info.MatPath{5} = 'map.header.stamp.sec';
info.MatPath{6} = 'map.header.stamp.nsec';
info.MatPath{7} = 'map.header.frame_id';
info.MatPath{8} = 'map.binary';
info.MatPath{9} = 'map.id';
info.MatPath{10} = 'map.resolution';
info.MatPath{11} = 'map.data';
