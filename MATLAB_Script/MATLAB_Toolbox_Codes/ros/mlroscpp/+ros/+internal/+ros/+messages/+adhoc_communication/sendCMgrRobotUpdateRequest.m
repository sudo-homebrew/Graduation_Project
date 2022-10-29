function [data, info] = sendCMgrRobotUpdateRequest
%SendCMgrRobotUpdate gives an empty data for adhoc_communication/SendCMgrRobotUpdateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendCMgrRobotUpdateRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Update, info.Update] = ros.internal.ros.messages.adhoc_communication.cMgrRobotUpdate;
info.Update.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendCMgrRobotUpdateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'update';
info.MatPath{4} = 'update.robot_name';
info.MatPath{5} = 'update.capabilities';
info.MatPath{6} = 'update.energy';
info.MatPath{7} = 'update.dimensions';
info.MatPath{8} = 'update.dimensions.x';
info.MatPath{9} = 'update.dimensions.y';
info.MatPath{10} = 'update.dimensions.z';
info.MatPath{11} = 'update.status';
info.MatPath{12} = 'update.neighbors';
