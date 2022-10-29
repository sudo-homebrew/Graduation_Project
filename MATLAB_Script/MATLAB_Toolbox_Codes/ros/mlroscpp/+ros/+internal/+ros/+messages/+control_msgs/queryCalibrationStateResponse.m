function [data, info] = queryCalibrationStateResponse
%QueryCalibrationState gives an empty data for control_msgs/QueryCalibrationStateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/QueryCalibrationStateResponse';
[data.IsCalibrated, info.IsCalibrated] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'control_msgs/QueryCalibrationStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'is_calibrated';
