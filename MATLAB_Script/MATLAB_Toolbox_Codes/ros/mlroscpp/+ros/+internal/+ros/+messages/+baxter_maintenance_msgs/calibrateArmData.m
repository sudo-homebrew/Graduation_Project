function [data, info] = calibrateArmData
%CalibrateArmData gives an empty data for baxter_maintenance_msgs/CalibrateArmData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_maintenance_msgs/CalibrateArmData';
[data.SuppressWriteToFile, info.SuppressWriteToFile] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'baxter_maintenance_msgs/CalibrateArmData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'suppressWriteToFile';
