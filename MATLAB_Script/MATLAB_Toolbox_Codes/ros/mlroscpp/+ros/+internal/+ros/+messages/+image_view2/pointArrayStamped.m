function [data, info] = pointArrayStamped
%PointArrayStamped gives an empty data for image_view2/PointArrayStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_view2/PointArrayStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Points, info.Points] = ros.internal.ros.messages.geometry_msgs.point;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
info.MessageType = 'image_view2/PointArrayStamped';
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
info.MatPath{7} = 'points';
info.MatPath{8} = 'points.x';
info.MatPath{9} = 'points.y';
info.MatPath{10} = 'points.z';
