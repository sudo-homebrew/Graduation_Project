function [data, info] = setFollowStateRequest
%SetFollowState gives an empty data for turtlebot_msgs/SetFollowStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlebot_msgs/SetFollowStateRequest';
[data.STOPPED, info.STOPPED] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.FOLLOW, info.FOLLOW] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'turtlebot_msgs/SetFollowStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'STOPPED';
info.MatPath{2} = 'FOLLOW';
info.MatPath{3} = 'state';
