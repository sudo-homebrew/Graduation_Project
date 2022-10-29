function [data, info] = singleJointPositionGoal
%SingleJointPositionGoal gives an empty data for pr2_controllers_msgs/SingleJointPositionGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_controllers_msgs/SingleJointPositionGoal';
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MinDuration, info.MinDuration] = ros.internal.ros.messages.ros.duration;
info.MinDuration.MLdataType = 'struct';
[data.MaxVelocity, info.MaxVelocity] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_controllers_msgs/SingleJointPositionGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'position';
info.MatPath{2} = 'min_duration';
info.MatPath{3} = 'min_duration.sec';
info.MatPath{4} = 'min_duration.nsec';
info.MatPath{5} = 'max_velocity';
