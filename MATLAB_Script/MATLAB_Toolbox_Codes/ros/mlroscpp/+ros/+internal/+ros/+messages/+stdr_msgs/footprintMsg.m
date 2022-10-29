function [data, info] = footprintMsg
%FootprintMsg gives an empty data for stdr_msgs/FootprintMsg

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/FootprintMsg';
[data.Points, info.Points] = ros.internal.ros.messages.geometry_msgs.point;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
[data.Radius, info.Radius] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'stdr_msgs/FootprintMsg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'points';
info.MatPath{2} = 'points.x';
info.MatPath{3} = 'points.y';
info.MatPath{4} = 'points.z';
info.MatPath{5} = 'radius';
