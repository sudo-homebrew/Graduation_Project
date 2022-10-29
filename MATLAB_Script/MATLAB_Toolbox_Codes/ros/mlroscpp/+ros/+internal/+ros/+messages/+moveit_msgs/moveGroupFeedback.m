function [data, info] = moveGroupFeedback
%MoveGroupFeedback gives an empty data for moveit_msgs/MoveGroupFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/MoveGroupFeedback';
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'moveit_msgs/MoveGroupFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'state';
