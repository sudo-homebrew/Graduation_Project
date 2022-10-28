function [data, info] = calibrateArmEnable
%CalibrateArmEnable gives an empty data for baxter_maintenance_msgs/CalibrateArmEnable

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_maintenance_msgs/CalibrateArmEnable';
[data.IsEnabled, info.IsEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Uid, info.Uid] = ros.internal.ros.messages.ros.char('string',0);
[data.Data, info.Data] = ros.internal.ros.messages.baxter_maintenance_msgs.calibrateArmData;
info.Data.MLdataType = 'struct';
info.MessageType = 'baxter_maintenance_msgs/CalibrateArmEnable';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'isEnabled';
info.MatPath{2} = 'uid';
info.MatPath{3} = 'data';
info.MatPath{4} = 'data.suppressWriteToFile';
