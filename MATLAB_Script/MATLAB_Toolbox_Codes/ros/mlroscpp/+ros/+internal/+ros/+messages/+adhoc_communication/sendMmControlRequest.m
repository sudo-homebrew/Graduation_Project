function [data, info] = sendMmControlRequest
%SendMmControl gives an empty data for adhoc_communication/SendMmControlRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendMmControlRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Msg, info.Msg] = ros.internal.ros.messages.adhoc_communication.mmControl;
info.Msg.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendMmControlRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'msg';
info.MatPath{4} = 'msg.src_robot';
info.MatPath{5} = 'msg.update_numbers';
