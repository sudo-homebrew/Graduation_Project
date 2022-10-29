function [data, info] = calibrateRequest
%calibrate gives an empty data for visp_camera_calibration/calibrateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_camera_calibration/calibrateRequest';
[data.Method, info.Method] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.SampleWidth, info.SampleWidth] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.SampleHeight, info.SampleHeight] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'visp_camera_calibration/calibrateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'method';
info.MatPath{2} = 'sample_width';
info.MatPath{3} = 'sample_height';
