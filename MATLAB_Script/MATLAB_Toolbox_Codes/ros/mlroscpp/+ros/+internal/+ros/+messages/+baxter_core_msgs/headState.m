function [data, info] = headState
%HeadState gives an empty data for baxter_core_msgs/HeadState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/HeadState';
[data.Pan, info.Pan] = ros.internal.ros.messages.ros.default_type('single',1);
[data.IsTurning, info.IsTurning] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.IsNodding, info.IsNodding] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.IsPanEnabled, info.IsPanEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'baxter_core_msgs/HeadState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'pan';
info.MatPath{2} = 'isTurning';
info.MatPath{3} = 'isNodding';
info.MatPath{4} = 'isPanEnabled';
