function [data, info] = denseLaserPoint
%DenseLaserPoint gives an empty data for calibration_msgs/DenseLaserPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'calibration_msgs/DenseLaserPoint';
[data.Scan, info.Scan] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Ray, info.Ray] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'calibration_msgs/DenseLaserPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'scan';
info.MatPath{2} = 'ray';
