function [data, info] = ptuSetVelGoal
%PtuSetVelGoal gives an empty data for ptu_control/PtuSetVelGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ptu_control/PtuSetVelGoal';
[data.PanVel, info.PanVel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TiltVel, info.TiltVel] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'ptu_control/PtuSetVelGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'pan_vel';
info.MatPath{2} = 'tilt_vel';
