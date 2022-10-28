function [data, info] = linkStatesStamped
%LinkStatesStamped gives an empty data for multimaster_msgs_fkie/LinkStatesStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/LinkStatesStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Links, info.Links] = ros.internal.ros.messages.multimaster_msgs_fkie.linkState;
info.Links.MLdataType = 'struct';
info.Links.MaxLen = NaN;
info.Links.MinLen = 0;
data.Links = data.Links([],1);
info.MessageType = 'multimaster_msgs_fkie/LinkStatesStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'links';
info.MatPath{8} = 'links.destination';
info.MatPath{9} = 'links.quality';
