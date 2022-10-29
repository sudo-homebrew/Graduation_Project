function [data, info] = goPosGoal
%GoPosGoal gives an empty data for jsk_footstep_controller/GoPosGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_footstep_controller/GoPosGoal';
[data.NEWTARGET, info.NEWTARGET] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.OVERWRITE, info.OVERWRITE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.ABSOLUTENEWTARGET, info.ABSOLUTENEWTARGET] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.ABSOLUTEOVERWRITE, info.ABSOLUTEOVERWRITE] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Theta, info.Theta] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Action, info.Action] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'jsk_footstep_controller/GoPosGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'NEW_TARGET';
info.MatPath{2} = 'OVER_WRITE';
info.MatPath{3} = 'ABSOLUTE_NEW_TARGET';
info.MatPath{4} = 'ABSOLUTE_OVER_WRITE';
info.MatPath{5} = 'x';
info.MatPath{6} = 'y';
info.MatPath{7} = 'theta';
info.MatPath{8} = 'action';
