function [data, info] = turtlebotMoveGoal
%TurtlebotMoveGoal gives an empty data for turtlebot_actions/TurtlebotMoveGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlebot_actions/TurtlebotMoveGoal';
[data.TurnDistance, info.TurnDistance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ForwardDistance, info.ForwardDistance] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'turtlebot_actions/TurtlebotMoveGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'turn_distance';
info.MatPath{2} = 'forward_distance';
