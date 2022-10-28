function [data, info] = kltPoints
%KltPoints gives an empty data for visp_tracker/KltPoints

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/KltPoints';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.KltPointsPositions, info.KltPointsPositions] = ros.internal.ros.messages.visp_tracker.kltPoint;
info.KltPointsPositions.MLdataType = 'struct';
info.KltPointsPositions.MaxLen = NaN;
info.KltPointsPositions.MinLen = 0;
data.KltPointsPositions = data.KltPointsPositions([],1);
info.MessageType = 'visp_tracker/KltPoints';
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
info.MatPath{7} = 'klt_points_positions';
info.MatPath{8} = 'klt_points_positions.i';
info.MatPath{9} = 'klt_points_positions.j';
info.MatPath{10} = 'klt_points_positions.id';
