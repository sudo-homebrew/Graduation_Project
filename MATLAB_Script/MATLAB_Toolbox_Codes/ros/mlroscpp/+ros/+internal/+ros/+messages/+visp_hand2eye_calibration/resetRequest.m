function [data, info] = resetRequest
%reset gives an empty data for visp_hand2eye_calibration/resetRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_hand2eye_calibration/resetRequest';
info.MessageType = 'visp_hand2eye_calibration/resetRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
