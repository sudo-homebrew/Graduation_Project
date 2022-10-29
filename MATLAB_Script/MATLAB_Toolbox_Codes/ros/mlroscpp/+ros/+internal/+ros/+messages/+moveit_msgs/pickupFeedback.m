function [data, info] = pickupFeedback
%PickupFeedback gives an empty data for moveit_msgs/PickupFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/PickupFeedback';
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'moveit_msgs/PickupFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'state';
