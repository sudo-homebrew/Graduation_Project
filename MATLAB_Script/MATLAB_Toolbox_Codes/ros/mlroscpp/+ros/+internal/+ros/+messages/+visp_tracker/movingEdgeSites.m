function [data, info] = movingEdgeSites
%MovingEdgeSites gives an empty data for visp_tracker/MovingEdgeSites

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/MovingEdgeSites';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.MovingEdgeSites_, info.MovingEdgeSites_] = ros.internal.ros.messages.visp_tracker.movingEdgeSite;
info.MovingEdgeSites_.MLdataType = 'struct';
info.MovingEdgeSites_.MaxLen = NaN;
info.MovingEdgeSites_.MinLen = 0;
data.MovingEdgeSites_ = data.MovingEdgeSites_([],1);
info.MessageType = 'visp_tracker/MovingEdgeSites';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'moving_edge_sites';
info.MatPath{8} = 'moving_edge_sites.x';
info.MatPath{9} = 'moving_edge_sites.y';
info.MatPath{10} = 'moving_edge_sites.suppress';
