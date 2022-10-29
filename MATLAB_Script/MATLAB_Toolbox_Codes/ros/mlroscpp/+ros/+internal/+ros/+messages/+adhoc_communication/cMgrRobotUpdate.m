function [data, info] = cMgrRobotUpdate
%CMgrRobotUpdate gives an empty data for adhoc_communication/CMgrRobotUpdate

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/CMgrRobotUpdate';
[data.RobotName, info.RobotName] = ros.internal.ros.messages.ros.char('string',0);
[data.Capabilities, info.Capabilities] = ros.internal.ros.messages.ros.char('string',0);
[data.Energy, info.Energy] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Dimensions, info.Dimensions] = ros.internal.ros.messages.adhoc_communication.cMgrDimensions;
info.Dimensions.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
[data.Neighbors, info.Neighbors] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'adhoc_communication/CMgrRobotUpdate';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'robot_name';
info.MatPath{2} = 'capabilities';
info.MatPath{3} = 'energy';
info.MatPath{4} = 'dimensions';
info.MatPath{5} = 'dimensions.x';
info.MatPath{6} = 'dimensions.y';
info.MatPath{7} = 'dimensions.z';
info.MatPath{8} = 'status';
info.MatPath{9} = 'neighbors';
