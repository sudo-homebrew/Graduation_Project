function [data, info] = getSnapshotGoal
%GetSnapshotGoal gives an empty data for pr2_tilt_laser_interface/GetSnapshotGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_tilt_laser_interface/GetSnapshotGoal';
[data.StartAngle, info.StartAngle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.EndAngle, info.EndAngle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Speed, info.Speed] = ros.internal.ros.messages.ros.default_type('single',1);
[data.HiFidelity, info.HiFidelity] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Continuous, info.Continuous] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'pr2_tilt_laser_interface/GetSnapshotGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'start_angle';
info.MatPath{2} = 'end_angle';
info.MatPath{3} = 'speed';
info.MatPath{4} = 'hi_fidelity';
info.MatPath{5} = 'continuous';
