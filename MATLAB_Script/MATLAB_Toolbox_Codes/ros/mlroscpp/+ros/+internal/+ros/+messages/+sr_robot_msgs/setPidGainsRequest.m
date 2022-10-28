function [data, info] = setPidGainsRequest
%SetPidGains gives an empty data for sr_robot_msgs/SetPidGainsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/SetPidGainsRequest';
[data.P, info.P] = ros.internal.ros.messages.ros.default_type('double',1);
[data.I, info.I] = ros.internal.ros.messages.ros.default_type('double',1);
[data.D, info.D] = ros.internal.ros.messages.ros.default_type('double',1);
[data.IClamp, info.IClamp] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxForce, info.MaxForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Deadband, info.Deadband] = ros.internal.ros.messages.ros.default_type('double',1);
[data.FrictionDeadband, info.FrictionDeadband] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'sr_robot_msgs/SetPidGainsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'p';
info.MatPath{2} = 'i';
info.MatPath{3} = 'd';
info.MatPath{4} = 'i_clamp';
info.MatPath{5} = 'max_force';
info.MatPath{6} = 'deadband';
info.MatPath{7} = 'friction_deadband';
