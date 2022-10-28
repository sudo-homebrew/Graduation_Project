function [data, info] = takeSnapshotRequest
%TakeSnapshot gives an empty data for r2_msgs/TakeSnapshotRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/TakeSnapshotRequest';
[data.Cmd, info.Cmd] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'r2_msgs/TakeSnapshotRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'cmd';
