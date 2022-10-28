function [data, info] = setEffortControllerGainsRequest
%SetEffortControllerGains gives an empty data for sr_robot_msgs/SetEffortControllerGainsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/SetEffortControllerGainsRequest';
[data.MaxForce, info.MaxForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.FrictionDeadband, info.FrictionDeadband] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'sr_robot_msgs/SetEffortControllerGainsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'max_force';
info.MatPath{2} = 'friction_deadband';
