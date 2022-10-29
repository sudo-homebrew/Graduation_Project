function [data, info] = calibrateResponse
%calibrate gives an empty data for visp_camera_calibration/calibrateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_camera_calibration/calibrateResponse';
[data.StdDevErrs, info.StdDevErrs] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'visp_camera_calibration/calibrateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'stdDevErrs';
