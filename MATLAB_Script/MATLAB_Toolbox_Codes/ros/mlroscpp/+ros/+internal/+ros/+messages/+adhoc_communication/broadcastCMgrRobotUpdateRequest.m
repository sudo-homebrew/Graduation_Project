function [data, info] = broadcastCMgrRobotUpdateRequest
%BroadcastCMgrRobotUpdate gives an empty data for adhoc_communication/BroadcastCMgrRobotUpdateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/BroadcastCMgrRobotUpdateRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Update, info.Update] = ros.internal.ros.messages.adhoc_communication.cMgrRobotUpdate;
info.Update.MLdataType = 'struct';
[data.HopLimit, info.HopLimit] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'adhoc_communication/BroadcastCMgrRobotUpdateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'update';
info.MatPath{3} = 'update.robot_name';
info.MatPath{4} = 'update.capabilities';
info.MatPath{5} = 'update.energy';
info.MatPath{6} = 'update.dimensions';
info.MatPath{7} = 'update.dimensions.x';
info.MatPath{8} = 'update.dimensions.y';
info.MatPath{9} = 'update.dimensions.z';
info.MatPath{10} = 'update.status';
info.MatPath{11} = 'update.neighbors';
info.MatPath{12} = 'hop_limit';
