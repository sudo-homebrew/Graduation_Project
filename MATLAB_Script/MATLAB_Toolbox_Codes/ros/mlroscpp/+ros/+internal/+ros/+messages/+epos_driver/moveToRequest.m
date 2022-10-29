function [data, info] = moveToRequest
%MoveTo gives an empty data for epos_driver/MoveToRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'epos_driver/MoveToRequest';
[data.Pose, info.Pose] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'epos_driver/MoveToRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'pose';
