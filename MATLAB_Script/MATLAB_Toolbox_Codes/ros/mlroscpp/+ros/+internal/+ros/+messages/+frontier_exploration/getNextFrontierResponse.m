function [data, info] = getNextFrontierResponse
%GetNextFrontier gives an empty data for frontier_exploration/GetNextFrontierResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'frontier_exploration/GetNextFrontierResponse';
[data.NextFrontier, info.NextFrontier] = ros.internal.ros.messages.geometry_msgs.poseStamped;
info.NextFrontier.MLdataType = 'struct';
info.MessageType = 'frontier_exploration/GetNextFrontierResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'next_frontier';
info.MatPath{2} = 'next_frontier.header';
info.MatPath{3} = 'next_frontier.header.seq';
info.MatPath{4} = 'next_frontier.header.stamp';
info.MatPath{5} = 'next_frontier.header.stamp.sec';
info.MatPath{6} = 'next_frontier.header.stamp.nsec';
info.MatPath{7} = 'next_frontier.header.frame_id';
info.MatPath{8} = 'next_frontier.pose';
info.MatPath{9} = 'next_frontier.pose.position';
info.MatPath{10} = 'next_frontier.pose.position.x';
info.MatPath{11} = 'next_frontier.pose.position.y';
info.MatPath{12} = 'next_frontier.pose.position.z';
info.MatPath{13} = 'next_frontier.pose.orientation';
info.MatPath{14} = 'next_frontier.pose.orientation.x';
info.MatPath{15} = 'next_frontier.pose.orientation.y';
info.MatPath{16} = 'next_frontier.pose.orientation.z';
info.MatPath{17} = 'next_frontier.pose.orientation.w';
