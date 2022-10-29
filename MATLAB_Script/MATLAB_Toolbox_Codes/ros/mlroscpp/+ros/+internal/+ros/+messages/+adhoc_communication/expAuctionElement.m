function [data, info] = expAuctionElement
%ExpAuctionElement gives an empty data for adhoc_communication/ExpAuctionElement

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/ExpAuctionElement';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.DetectedByRobotStr, info.DetectedByRobotStr] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'adhoc_communication/ExpAuctionElement';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'id';
info.MatPath{2} = 'detected_by_robot_str';
