function [data, info] = getPeerListResponse
%GetPeerList gives an empty data for rtt_ros_msgs/GetPeerListResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rtt_ros_msgs/GetPeerListResponse';
[data.Peers, info.Peers] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rtt_ros_msgs/GetPeerListResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'peers';
