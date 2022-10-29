function [data, info] = updateBoundaryPolygonRequest
%UpdateBoundaryPolygon gives an empty data for frontier_exploration/UpdateBoundaryPolygonRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'frontier_exploration/UpdateBoundaryPolygonRequest';
[data.ExploreBoundary, info.ExploreBoundary] = ros.internal.ros.messages.geometry_msgs.polygonStamped;
info.ExploreBoundary.MLdataType = 'struct';
info.MessageType = 'frontier_exploration/UpdateBoundaryPolygonRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'explore_boundary';
info.MatPath{2} = 'explore_boundary.header';
info.MatPath{3} = 'explore_boundary.header.seq';
info.MatPath{4} = 'explore_boundary.header.stamp';
info.MatPath{5} = 'explore_boundary.header.stamp.sec';
info.MatPath{6} = 'explore_boundary.header.stamp.nsec';
info.MatPath{7} = 'explore_boundary.header.frame_id';
info.MatPath{8} = 'explore_boundary.polygon';
info.MatPath{9} = 'explore_boundary.polygon.points';
info.MatPath{10} = 'explore_boundary.polygon.points.x';
info.MatPath{11} = 'explore_boundary.polygon.points.y';
info.MatPath{12} = 'explore_boundary.polygon.points.z';
