function [data, info] = spawnRobotFeedback
%SpawnRobotFeedback gives an empty data for stdr_msgs/SpawnRobotFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/SpawnRobotFeedback';
info.MessageType = 'stdr_msgs/SpawnRobotFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);