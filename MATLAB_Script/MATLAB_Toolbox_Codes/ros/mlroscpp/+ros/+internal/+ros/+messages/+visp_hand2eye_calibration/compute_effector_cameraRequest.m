function [data, info] = compute_effector_cameraRequest
%compute_effector_camera gives an empty data for visp_hand2eye_calibration/compute_effector_cameraRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_hand2eye_calibration/compute_effector_cameraRequest';
info.MessageType = 'visp_hand2eye_calibration/compute_effector_cameraRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
