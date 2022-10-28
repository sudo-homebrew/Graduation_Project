function [data, info] = poseCommandStatus
%PoseCommandStatus gives an empty data for r2_msgs/PoseCommandStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/PoseCommandStatus';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.CommandId, info.CommandId] = ros.internal.ros.messages.ros.char('string',0);
[data.PENDING, info.PENDING] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ACTIVE, info.ACTIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.REJECTED, info.REJECTED] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.SUCCEEDED, info.SUCCEEDED] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.FAILED, info.FAILED] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'r2_msgs/PoseCommandStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'commandId';
info.MatPath{5} = 'PENDING';
info.MatPath{6} = 'ACTIVE';
info.MatPath{7} = 'REJECTED';
info.MatPath{8} = 'SUCCEEDED';
info.MatPath{9} = 'FAILED';
info.MatPath{10} = 'status';
info.MatPath{11} = 'statusMessage';
