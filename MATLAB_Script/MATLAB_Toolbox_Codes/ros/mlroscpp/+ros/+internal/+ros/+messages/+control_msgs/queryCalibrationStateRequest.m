function [data, info] = queryCalibrationStateRequest
%QueryCalibrationState gives an empty data for control_msgs/QueryCalibrationStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/QueryCalibrationStateRequest';
info.MessageType = 'control_msgs/QueryCalibrationStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);