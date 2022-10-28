function [data, info] = setFollowStateResponse
%SetFollowState gives an empty data for turtlebot_msgs/SetFollowStateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlebot_msgs/SetFollowStateResponse';
[data.OK, info.OK] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'turtlebot_msgs/SetFollowStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'OK';
info.MatPath{2} = 'ERROR';
info.MatPath{3} = 'result';
