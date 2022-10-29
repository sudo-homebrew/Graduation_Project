function [data, info] = calibPoint
%CalibPoint gives an empty data for visp_camera_calibration/CalibPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_camera_calibration/CalibPoint';
[data.I, info.I] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.J, info.J] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'visp_camera_calibration/CalibPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'i';
info.MatPath{2} = 'j';
info.MatPath{3} = 'X';
info.MatPath{4} = 'Y';
info.MatPath{5} = 'Z';
