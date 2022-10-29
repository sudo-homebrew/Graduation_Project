function [data, info] = sendQuaternionRequest
%SendQuaternion gives an empty data for adhoc_communication/SendQuaternionRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendQuaternionRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Quaternion, info.Quaternion] = ros.internal.ros.messages.geometry_msgs.quaternion;
info.Quaternion.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendQuaternionRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'quaternion';
info.MatPath{4} = 'quaternion.x';
info.MatPath{5} = 'quaternion.y';
info.MatPath{6} = 'quaternion.z';
info.MatPath{7} = 'quaternion.w';
