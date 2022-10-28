function [data, info] = sendMmPointRequest
%SendMmPoint gives an empty data for adhoc_communication/SendMmPointRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendMmPointRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Point, info.Point] = ros.internal.ros.messages.adhoc_communication.mmPoint;
info.Point.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendMmPointRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'point';
info.MatPath{4} = 'point.src_robot';
info.MatPath{5} = 'point.x';
info.MatPath{6} = 'point.y';
