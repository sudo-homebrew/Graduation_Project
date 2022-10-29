function [data, info] = calibPointArray
%CalibPointArray gives an empty data for visp_camera_calibration/CalibPointArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_camera_calibration/CalibPointArray';
[data.Points, info.Points] = ros.internal.ros.messages.visp_camera_calibration.calibPoint;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
info.MessageType = 'visp_camera_calibration/CalibPointArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'points';
info.MatPath{2} = 'points.i';
info.MatPath{3} = 'points.j';
info.MatPath{4} = 'points.X';
info.MatPath{5} = 'points.Y';
info.MatPath{6} = 'points.Z';
