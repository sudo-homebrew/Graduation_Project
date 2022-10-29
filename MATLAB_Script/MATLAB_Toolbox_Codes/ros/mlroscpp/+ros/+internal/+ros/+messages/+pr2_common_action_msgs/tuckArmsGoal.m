function [data, info] = tuckArmsGoal
%TuckArmsGoal gives an empty data for pr2_common_action_msgs/TuckArmsGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_common_action_msgs/TuckArmsGoal';
[data.TuckLeft, info.TuckLeft] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.TuckRight, info.TuckRight] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'pr2_common_action_msgs/TuckArmsGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'tuck_left';
info.MatPath{2} = 'tuck_right';
