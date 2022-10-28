function [data, info] = ptuGotoGoal
%PtuGotoGoal gives an empty data for ptu_control/PtuGotoGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ptu_control/PtuGotoGoal';
[data.Pan, info.Pan] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Tilt, info.Tilt] = ros.internal.ros.messages.ros.default_type('single',1);
[data.PanVel, info.PanVel] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TiltVel, info.TiltVel] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'ptu_control/PtuGotoGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'pan';
info.MatPath{2} = 'tilt';
info.MatPath{3} = 'pan_vel';
info.MatPath{4} = 'tilt_vel';
