function [data, info] = moveCycleRequest
%MoveCycle gives an empty data for epos_driver/MoveCycleRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'epos_driver/MoveCycleRequest';
[data.PoseTop, info.PoseTop] = ros.internal.ros.messages.ros.default_type('double',1);
[data.PoseBottom, info.PoseBottom] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'epos_driver/MoveCycleRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'pose_top';
info.MatPath{2} = 'pose_bottom';
