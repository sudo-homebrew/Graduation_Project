function [data, info] = forceControllerResponse
%ForceController gives an empty data for sr_robot_msgs/ForceControllerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ForceControllerResponse';
[data.Configured, info.Configured] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'sr_robot_msgs/ForceControllerResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'configured';
